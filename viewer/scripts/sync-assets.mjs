// Copies the FreeCAD export (STLs, part masses) and the MuJoCo results into
// public/assets so the viewer always shows the latest build.
import { cpSync, existsSync, mkdirSync, readdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const build = join(here, "..", "..", "build");
const out = join(here, "..", "public", "assets");
mkdirSync(join(out, "stl"), { recursive: true });

for (const f of readdirSync(join(build, "stl")).filter((f) => f.endsWith(".stl"))) {
  cpSync(join(build, "stl", f), join(out, "stl", f));
}
for (const f of ["parts.json", "results.json", "forces.json"]) {
  if (existsSync(join(build, f))) cpSync(join(build, f), join(out, f));
}
console.log("synced assets from", build);
