// Prototyping agent — runs inside a GitHub Actions runner.
// The repo is already checked out on the correct branch.
// This script: plans changes, generates code, writes files, runs build/test,
// and retries on failure. The workflow handles committing and pushing.

import { execSync } from "node:child_process";
import { readFileSync, writeFileSync, mkdirSync, existsSync, readdirSync, statSync } from "node:fs";
import { dirname, join, relative } from "node:path";

// --- Config from env ---

const API_KEY = process.env.MINIMAX_API_KEY;
const API_BASE = process.env.MINIMAX_API_BASE || "https://api.minimax.io";
const MODEL = process.env.MINIMAX_MODEL || "MiniMax-M2.5";
const TITLE = process.env.FEATURE_TITLE || "";
const DESCRIPTION = process.env.FEATURE_DESCRIPTION || "";
const AREA = process.env.FEATURE_AREA || "";

if (!API_KEY) {
  console.error("MINIMAX_API_KEY is not set");
  process.exit(1);
}
if (!TITLE) {
  console.error("FEATURE_TITLE is not set");
  process.exit(1);
}

const MAX_FILES = 6;
const MAX_FILE_READ = 12_000;
const MAX_RETRIES = 2;

// --- LLM helper ---

interface ChatCompletionResponse {
  choices?: { message?: { content?: string } }[];
}

async function callLLM(prompt: string): Promise<string> {
  const res = await fetch(`${API_BASE}/v1/text/chatcompletion_v2`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      Authorization: `Bearer ${API_KEY}`,
    },
    body: JSON.stringify({
      model: MODEL,
      messages: [{ role: "user", content: prompt }],
      max_tokens: 4096,
      temperature: 0.3,
    }),
  });

  if (!res.ok) {
    const text = await res.text();
    throw new Error(`LLM call failed (${res.status}): ${text}`);
  }

  const data = (await res.json()) as ChatCompletionResponse;
  return data.choices?.[0]?.message?.content ?? "";
}

// --- Repo helpers ---

function listFiles(dir: string, base: string = dir): string[] {
  const entries: string[] = [];
  for (const name of readdirSync(dir)) {
    if (name === "node_modules" || name === ".git" || name === "dist") continue;
    const full = join(dir, name);
    const rel = relative(base, full);
    if (statSync(full).isDirectory()) {
      entries.push(...listFiles(full, base));
    } else {
      entries.push(rel);
    }
  }
  return entries;
}

function readFileSafe(path: string): string {
  try {
    const content = readFileSync(path, "utf-8");
    return content.length > MAX_FILE_READ
      ? content.slice(0, MAX_FILE_READ) + "\n// ... (truncated)"
      : content;
  } catch {
    return "";
  }
}

function runBuild(): { ok: boolean; output: string } {
  try {
    const output = execSync("npm run build 2>&1 || npm run typecheck 2>&1", {
      encoding: "utf-8",
      timeout: 60_000,
      cwd: process.cwd(),
    });
    return { ok: true, output };
  } catch (err: any) {
    return { ok: false, output: err.stdout || err.message || String(err) };
  }
}

// --- Planning ---

interface PlannedChange {
  path: string;
  action: "create" | "modify";
  description: string;
}

async function plan(tree: string[]): Promise<PlannedChange[]> {
  const prompt = `You are a senior engineer planning a minimal prototype implementation.

FEATURE REQUEST:
Title: ${TITLE}
Description: ${DESCRIPTION}
Affected area: ${AREA}

REPOSITORY FILES:
${tree.join("\n")}

Decide which files to CREATE or MODIFY. Keep the change minimal — a working prototype, not production code. Max ${MAX_FILES} files.

Respond with STRICT JSON, no markdown:
{
  "changes": [
    {"path": "src/example.ts", "action": "create" | "modify", "description": "what to change"}
  ]
}

If you cannot reasonably prototype this, return {"changes": []}.`;

  const raw = await callLLM(prompt);
  try {
    const cleaned = raw.replace(/```json|```/g, "").trim();
    const parsed = JSON.parse(cleaned) as { changes: PlannedChange[] };
    return (parsed.changes || [])
      .filter(
        (c) =>
          typeof c.path === "string" &&
          (c.action === "create" || c.action === "modify") &&
          typeof c.description === "string"
      )
      .slice(0, MAX_FILES);
  } catch {
    console.error("Could not parse plan:", raw);
    return [];
  }
}

// --- Code generation ---

async function generateFile(
  change: PlannedChange,
  currentContent: string
): Promise<string | null> {
  const context =
    change.action === "modify" && currentContent
      ? `\nCURRENT FILE CONTENT:\n\`\`\`\n${currentContent}\n\`\`\``
      : "";

  const prompt = `You are implementing a feature. Output ONLY the complete file content — no markdown fences, no explanation.

FEATURE REQUEST:
Title: ${TITLE}
Description: ${DESCRIPTION}

FILE: ${change.path}
ACTION: ${change.action}
TASK: ${change.description}
${context}

Rules:
- Output the ENTIRE file content (not a diff).
- Keep changes minimal and focused.
- Follow existing code style.
- Do not add unrelated changes.`;

  const raw = await callLLM(prompt);
  const content = raw
    .replace(/^```[\w]*\n?/, "")
    .replace(/\n?```\s*$/, "")
    .trimEnd();
  return content || null;
}

// --- Fix loop ---

async function fixBuildError(
  errorOutput: string,
  filesChanged: string[]
): Promise<Map<string, string>> {
  const fileContents = filesChanged
    .map((f) => `--- ${f} ---\n${readFileSafe(f)}`)
    .join("\n\n");

  const prompt = `The build/typecheck failed after implementing a feature. Fix the errors.

BUILD ERROR:
${errorOutput.slice(0, 3000)}

FILES THAT WERE CHANGED:
${fileContents}

For each file that needs fixing, respond with STRICT JSON, no markdown:
{
  "fixes": [
    {"path": "src/example.ts", "content": "entire corrected file content"}
  ]
}

Only include files that need changes. Output complete file contents, not diffs.`;

  const raw = await callLLM(prompt);
  const fixes = new Map<string, string>();
  try {
    const cleaned = raw.replace(/```json|```/g, "").trim();
    const parsed = JSON.parse(cleaned) as {
      fixes: { path: string; content: string }[];
    };
    for (const fix of parsed.fixes || []) {
      if (typeof fix.path === "string" && typeof fix.content === "string") {
        fixes.set(fix.path, fix.content);
      }
    }
  } catch {
    console.error("Could not parse fix response:", raw);
  }
  return fixes;
}

// --- Main ---

async function main() {
  const root = process.cwd();
  console.log(`Agent: prototyping "${TITLE}"`);

  // 1. List repo files.
  const tree = listFiles(root);
  console.log(`Agent: ${tree.length} files in repo`);

  // 2. Plan.
  const changes = await plan(tree);
  if (!changes.length) {
    console.log("Agent: nothing to change — exiting");
    return;
  }
  console.log(`Agent: planned ${changes.length} file(s):`, changes.map((c) => c.path));

  // 3. Generate code for each file.
  const written: string[] = [];
  for (const change of changes) {
    const current =
      change.action === "modify" ? readFileSafe(join(root, change.path)) : "";
    const content = await generateFile(change, current);
    if (!content) {
      console.warn(`Agent: empty generation for ${change.path} — skipping`);
      continue;
    }
    const fullPath = join(root, change.path);
    mkdirSync(dirname(fullPath), { recursive: true });
    writeFileSync(fullPath, content + "\n");
    written.push(change.path);
    console.log(`Agent: wrote ${change.path}`);
  }

  if (!written.length) {
    console.log("Agent: no files written — exiting");
    return;
  }

  // 4. Build/test loop with retries.
  for (let attempt = 0; attempt <= MAX_RETRIES; attempt++) {
    const build = runBuild();
    if (build.ok) {
      console.log("Agent: build succeeded");
      return;
    }

    console.warn(`Agent: build failed (attempt ${attempt + 1}/${MAX_RETRIES + 1})`);
    if (attempt === MAX_RETRIES) {
      console.error("Agent: giving up after max retries. Changes are still on disk for review.");
      console.error("Build output:", build.output.slice(0, 2000));
      return;
    }

    // Ask LLM to fix.
    const fixes = await fixBuildError(build.output, written.map((f) => join(root, f)));
    if (!fixes.size) {
      console.error("Agent: LLM returned no fixes — giving up");
      return;
    }
    for (const [path, content] of fixes) {
      const fullPath = join(root, path);
      mkdirSync(dirname(fullPath), { recursive: true });
      writeFileSync(fullPath, content + "\n");
      console.log(`Agent: fixed ${path}`);
      if (!written.includes(path)) written.push(path);
    }
  }
}

main().catch((err) => {
  console.error("Agent failed:", err);
  process.exit(1);
});
