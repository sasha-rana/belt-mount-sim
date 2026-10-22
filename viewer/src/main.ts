// Interactive viewer for the FreeCAD belt-mount assembly (build/stl) with the
// MuJoCo pull-test results (build/results.json) overlaid as safety-factor colours.
// Units are mm; Z is the panel normal, pointing to the belt side.

import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { CSS2DRenderer } from "three/examples/jsm/renderers/CSS2DRenderer.js";
import { BODY_LABEL, ForceView, table, type Body, type ForcesFile } from "./forces";
import "./style.css";

type Kind = "inner-skin" | "core" | "outer-skin" | "spreader" | "bushing" | "annulus" | "backing" | "eyebolt" | "nut";

interface PartInfo { volume_cm3: number; mass_g: number }
interface Margin { stress: number; yield: number | null; ultimate: number | null }
interface CaseResult { description: string; tension_N: number; margins: Record<string, Margin> }

interface Part {
  name: string;
  group: "Panel" | "Lap mount" | "Anti-sub mount";
  kind: Kind;
  mesh: THREE.Mesh;
  material: THREE.MeshStandardMaterial;
  baseColor: THREE.Color;
  info?: PartInfo;
  row?: HTMLElement;
}

const PANEL_PARTS: Kind[] = ["inner-skin", "core", "outer-skin"];
const MOUNT_PARTS: Kind[] = ["spreader", "bushing", "annulus", "backing", "eyebolt", "nut"];

const STYLE: Record<Kind, { color: number; metal: number; rough: number; material: string }> = {
  "inner-skin": { color: 0x26282c, metal: 0.1, rough: 0.6, material: "CFRP skin 2.84 mm" },
  "outer-skin": { color: 0x26282c, metal: 0.1, rough: 0.6, material: "CFRP skin 2.84 mm" },
  core: { color: 0xd8c27a, metal: 0.0, rough: 0.9, material: "3.1 pcf Al honeycomb, 12 mm" },
  spreader: { color: 0xc4c9d0, metal: 0.8, rough: 0.35, material: "Aluminium" },
  bushing: { color: 0xa9b0b9, metal: 0.8, rough: 0.35, material: "Aluminium" },
  annulus: { color: 0xe3d3a3, metal: 0.0, rough: 0.8, material: "Syntactic foam" },
  backing: { color: 0xb7bcc4, metal: 0.8, rough: 0.35, material: "Aluminium" },
  eyebolt: { color: 0x2f7a55, metal: 0.6, rough: 0.4, material: "Steel (M12, long shank)" },
  nut: { color: 0x5b5d63, metal: 0.7, rough: 0.4, material: "Steel M12 nyloc" },
};

// Stack order for the exploded view (units of the explode distance)
const EXPLODE: Record<Kind, number> = {
  eyebolt: 3, spreader: 2, "outer-skin": 1, bushing: 0, annulus: 0, core: 0, "inner-skin": -1, backing: -2, nut: -3,
};
const EXPLODE_MM = 18;

// Which check in results.json colours which part (worst of the listed checks)
const SF_CHECKS: Partial<Record<Kind, string[]>> = {
  eyebolt: ["eyebolt shank, 8.8 steel", "eyebolt neck, 8.8 steel"],
  nut: ["nut threads (8 nut)"],
  backing: ["backing plate, 7050-T7451"],
  spreader: ["spreader, 7050-T7451"],
  bushing: ["bushing compression, 7050-T7451"],
  "inner-skin": ["inner skin punch-through (Ø55 perimeter)"],
  "outer-skin": ["skin bearing under pads"],
};

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;

// ---------- scene ----------
const viewport = $("viewport");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.localClippingEnabled = true;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
viewport.prepend(renderer.domElement);
const labelRenderer = new CSS2DRenderer();
labelRenderer.domElement.className = "labels";
viewport.append(labelRenderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xeef1f4);
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const camera = new THREE.PerspectiveCamera(35, 1, 1, 5000);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.screenSpacePanning = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 1.6));
const key = new THREE.DirectionalLight(0xffffff, 2.2);
key.position.set(200, -300, 400);
scene.add(key);
const fill = new THREE.DirectionalLight(0xffffff, 0.8);
fill.position.set(-300, 200, -200);
scene.add(fill);

const grid = new THREE.GridHelper(400, 40, 0xb5bcc6, 0xd5dae0);
grid.rotation.x = Math.PI / 2;
grid.position.z = -40;
scene.add(grid);
scene.add(new THREE.AxesHelper(30));

const forceView = new ForceView();
scene.add(forceView.group);
let forces: ForcesFile | undefined;

const clipPlane = new THREE.Plane(new THREE.Vector3(0, -1, 0), 0);
const parts: Part[] = [];
const root = new THREE.Group();
scene.add(root);

// ---------- loading ----------
function partList(): { name: string; group: Part["group"]; kind: Kind }[] {
  const list: { name: string; group: Part["group"]; kind: Kind }[] = PANEL_PARTS.map((k) => ({ name: k, group: "Panel", kind: k }));
  for (const [prefix, group] of [["lap", "Lap mount"], ["antisub", "Anti-sub mount"]] as const) {
    for (const k of MOUNT_PARTS) list.push({ name: `${prefix}-${k}`, group, kind: k });
  }
  return list;
}

async function fetchJson<T>(url: string): Promise<T | undefined> {
  try {
    const r = await fetch(url);
    return r.ok ? ((await r.json()) as T) : undefined;
  } catch {
    return undefined;
  }
}

let results: Record<string, CaseResult> = {};

async function load() {
  const loader = new STLLoader();
  const [info, res, frc] = await Promise.all([
    fetchJson<Record<string, PartInfo>>("assets/parts.json"),
    fetchJson<Record<string, CaseResult>>("assets/results.json"),
    fetchJson<ForcesFile>("assets/forces.json"),
  ]);
  forces = frc;
  results = Object.fromEntries(Object.entries(res ?? {}).filter(([, v]) => v && (v as CaseResult).margins));

  await Promise.all(partList().map(async ({ name, group, kind }) => {
    const geom = await loader.loadAsync(`assets/stl/${name}.stl`);
    geom.computeVertexNormals();
    const s = STYLE[kind];
    const material = new THREE.MeshStandardMaterial({
      color: s.color, metalness: s.metal, roughness: s.rough, side: THREE.DoubleSide, clippingPlanes: [],
    });
    const mesh = new THREE.Mesh(geom, material);
    mesh.userData.part = name;
    root.add(mesh);
    parts.push({ name, group, kind, mesh, material, baseColor: new THREE.Color(s.color), info: info?.[name] });
  }));
  parts.sort((a, b) => partList().findIndex((p) => p.name === a.name) - partList().findIndex((p) => p.name === b.name));

  buildTree();
  buildCases();
  applyPanelOpacity();
  setView("iso");
  updateForces();
}

// ---------- sidebar ----------
function buildTree() {
  const tree = $("tree");
  const groups = new Map<string, Part[]>();
  for (const p of parts) groups.set(p.group, [...(groups.get(p.group) ?? []), p]);
  for (const [group, list] of groups) {
    const g = document.createElement("div");
    g.className = "group";
    const head = document.createElement("label");
    head.className = "group-head";
    const all = document.createElement("input");
    all.type = "checkbox";
    all.checked = true;
    all.onchange = () => list.forEach((p) => setVisible(p, all.checked));
    head.append(all, group);
    g.append(head);
    for (const p of list) {
      const row = document.createElement("div");
      row.className = "row";
      const cb = document.createElement("input");
      cb.type = "checkbox";
      cb.checked = true;
      cb.onclick = (e) => e.stopPropagation();
      cb.onchange = () => setVisible(p, cb.checked);
      const sw = document.createElement("span");
      sw.className = "swatch";
      sw.style.background = `#${p.baseColor.getHexString()}`;
      const label = document.createElement("span");
      label.textContent = p.name.replace(/^(lap|antisub)-/, "");
      const mass = document.createElement("span");
      mass.className = "muted right";
      mass.textContent = p.info ? `${p.info.mass_g.toFixed(0)} g` : "";
      row.append(cb, sw, label, mass);
      row.onclick = () => select(p);
      row.ondblclick = () => focus(p);
      p.row = row;
      g.append(row);
    }
    tree.append(g);
  }
  const mount = parts.filter((p) => p.group === "Lap mount").reduce((s, p) => s + (p.info?.mass_g ?? 0), 0);
  $("totals").textContent = mount ? `Hardware per mount ≈ ${mount.toFixed(0)} g (both ≈ ${(2 * mount).toFixed(0)} g)` : "";
}

function setVisible(p: Part, v: boolean) {
  p.mesh.visible = v;
  const cb = p.row?.querySelector("input");
  if (cb) cb.checked = v;
}

function buildCases() {
  const sel = $<HTMLSelectElement>("loadCase");
  const names = new Map<string, string>();
  for (const [k, v] of Object.entries(forces?.cases ?? {})) names.set(k, v.description);
  for (const [k, v] of Object.entries(results)) names.set(k, v.description);
  for (const [k, d] of names) sel.append(new Option(d, k));
  if (!Object.keys(results).length) $<HTMLSelectElement>("colorMode").disabled = true;
  if (!names.size) sel.append(new Option("No results — run mujoco/analyze.py and export_forces.py"));
}

// ---------- colouring ----------
function sfFor(p: Part, c: CaseResult): { sf: number; check: string; m: Margin } | undefined {
  let worst: { sf: number; check: string; m: Margin } | undefined;
  for (const check of SF_CHECKS[p.kind] ?? []) {
    const m = c.margins[check];
    const allow = m?.yield ?? m?.ultimate;
    if (!m || !allow || m.stress <= 0) continue;
    const sf = allow / m.stress;
    if (!worst || sf < worst.sf) worst = { sf, check, m };
  }
  return worst;
}

function sfColor(sf: number): THREE.Color {
  // red (<1) -> amber (~1.5) -> green (>=3)
  const t = THREE.MathUtils.clamp((sf - 1) / 2, 0, 1);
  return new THREE.Color().setHSL((sf < 1 ? 0 : 0.02 + t * 0.31), 0.75, 0.5);
}

function recolor() {
  const mode = $<HTMLSelectElement>("colorMode").value;
  const c = results[$<HTMLSelectElement>("loadCase").value];
  $("legend").classList.toggle("hidden", mode !== "sf");
  for (const p of parts) {
    const r = mode === "sf" && c ? sfFor(p, c) : undefined;
    p.material.color.copy(r ? sfColor(r.sf) : mode === "sf" ? new THREE.Color(0xd0d4da) : p.baseColor);
  }
  if (selected) showInfo(selected);
}

function applyPanelOpacity() {
  const o = Number($<HTMLInputElement>("panelOpacity").value);
  for (const p of parts) {
    if (p.group !== "Panel") continue;
    const op = p.kind === "core" ? o * 0.5 : o;
    p.material.transparent = op < 1;
    p.material.opacity = op;
    p.material.depthWrite = op >= 1;
  }
}

// ---------- explode & section ----------
function applyExplode() {
  const e = Number($<HTMLInputElement>("explode").value);
  for (const p of parts) p.mesh.position.z = EXPLODE[p.kind] * EXPLODE_MM * e;
  updateForces();
}

// ---------- forces ----------
const BODY_OF: Record<Kind, Body> = {
  eyebolt: "eyebolt", nut: "nut", spreader: "spreader", backing: "backing",
  bushing: "plug", annulus: "plug", core: "panel", "inner-skin": "panel", "outer-skin": "panel",
};
const KIND_OF: Record<Body, Kind> = {
  eyebolt: "eyebolt", nut: "nut", spreader: "spreader", backing: "backing", plug: "bushing", panel: "core",
};

function updateForces() {
  const c = forces?.cases[$<HTMLSelectElement>("loadCase").value];
  const on = $<HTMLInputElement>("showForces").checked;
  forceView.group.visible = on;
  const tableEl = $("forceTable");
  const idx = Number($<HTMLInputElement>("loadFrac").value);
  const frame = c?.frames[Math.min(idx, (c?.frames.length ?? 1) - 1)];
  $("loadFracText").textContent = frame ? `${Math.round(frame.fraction * 100)}%` : "";
  if (!c || !frame) {
    tableEl.innerHTML = `<tr><td class="muted">No forces.json for this case — run mujoco/export_forces.py</td></tr>`;
    forceView.update({ frame: { fraction: 0, mounts: {} }, mounts: [], mmPerKN: 1, pads: false, explode: () => 0 });
    return;
  }
  const mountSel = $<HTMLSelectElement>("forceMount").value;
  const mounts = mountSel === "both" ? Object.keys(frame.mounts) : [mountSel];
  const body = selected ? BODY_OF[selected.kind] : undefined;
  const e = Number($<HTMLInputElement>("explode").value);
  forceView.update({
    frame, mounts, body,
    mmPerKN: Number($<HTMLInputElement>("arrowScale").value),
    pads: $<HTMLInputElement>("padForces").checked,
    explode: (b) => EXPLODE[KIND_OF[b]] * EXPLODE_MM * e,
  });
  if (!on) forceView.update({ frame, mounts: [], mmPerKN: 1, pads: false, explode: () => 0 });
  $("forceMode").textContent = body
    ? `Free body: ${BODY_LABEL[body]} (${mounts.join(" + ")}). Esc for the overview.`
    : "Overview. Click a part for its free-body diagram.";
  const mf = frame.mounts[mounts[0]];
  const rows = mf ? table(mf, body) : [];
  tableEl.innerHTML = rows.map((r, i) => `<tr class="${body && i === rows.length - 1 ? "sum" : ""}"><td>${
    r.color !== undefined ? `<span class="key" style="background:#${new THREE.Color(r.color).getHexString()}"></span>` : ""
  }${r.label}</td><td>${r.value}</td></tr>`).join("");
}

let playing: number | undefined;
function play() {
  const slider = $<HTMLInputElement>("loadFrac");
  if (playing !== undefined) {
    clearInterval(playing);
    playing = undefined;
    $("play").textContent = "▶ Ramp 0 → 100%";
    return;
  }
  slider.value = "0";
  updateForces();
  $("play").textContent = "■ Stop";
  playing = window.setInterval(() => {
    const v = Number(slider.value) + 1;
    if (v > Number(slider.max)) return play();
    slider.value = String(v);
    updateForces();
  }, 600);
}

function applySection() {
  const on = $<HTMLInputElement>("section").checked;
  clipPlane.constant = Number($<HTMLInputElement>("cut").value);
  for (const p of parts) p.material.clippingPlanes = on ? [clipPlane] : [];
}

// ---------- selection ----------
let selected: Part | undefined;
let hovered: Part | undefined;
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();

function pick(ev: PointerEvent | MouseEvent): Part | undefined {
  const r = renderer.domElement.getBoundingClientRect();
  pointer.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(pointer, camera);
  const on = $<HTMLInputElement>("section").checked;
  const hits = raycaster.intersectObjects(parts.filter((p) => p.mesh.visible).map((p) => p.mesh), false)
    .filter((h) => !on || clipPlane.distanceToPoint(h.point) >= 0);
  // Prefer mount hardware over the (often transparent) panel in front of it
  const hit = hits.find((h) => !PANEL_PARTS.includes(byMesh(h.object)!.kind)) ?? hits[0];
  return hit ? byMesh(hit.object) : undefined;
}

const byMesh = (o: THREE.Object3D) => parts.find((p) => p.mesh === o);

function highlight() {
  for (const p of parts) {
    p.material.emissive.setHex(p === selected ? 0x2a5bd7 : p === hovered ? 0x333333 : 0x000000);
    p.row?.classList.toggle("selected", p === selected);
  }
}

function select(p?: Part) {
  selected = p;
  highlight();
  if (p) showInfo(p);
  else $("info").classList.add("hidden");
  updateForces();
}

function showInfo(p: Part) {
  const info = $("info");
  const s = STYLE[p.kind];
  const lines = [`<h3>${p.name}</h3>`, `<div>${s.material}</div>`];
  if (p.info) lines.push(`<div>${p.info.volume_cm3.toFixed(2)} cm³ · ${p.info.mass_g.toFixed(1)} g</div>`);
  const c = results[$<HTMLSelectElement>("loadCase").value];
  const r = c ? sfFor(p, c) : undefined;
  if (r) {
    const allow = r.m.yield ?? r.m.ultimate;
    lines.push(`<div class="sf" style="border-color:#${sfColor(r.sf).getHexString()}">
      <b>${r.check}</b><br>${r.m.stress.toFixed(0)} MPa vs ${allow} MPa ${r.m.yield ? "yield" : "failure"}
      → SF <b>${r.sf.toFixed(2)}</b><br><span class="muted">${c.description}</span></div>`);
  }
  info.innerHTML = lines.join("");
  info.classList.remove("hidden");
}

// ---------- camera ----------
function bounds(objs: THREE.Object3D[]) {
  const box = new THREE.Box3();
  for (const o of objs) if (o.visible) box.expandByObject(o);
  return box;
}

function frame(box: THREE.Box3, dir?: THREE.Vector3) {
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const radius = box.getSize(new THREE.Vector3()).length() / 2;
  const d = dir ?? camera.position.clone().sub(controls.target).normalize();
  const dist = radius / Math.sin(THREE.MathUtils.degToRad(camera.fov / 2)) * 1.05;
  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(d, dist);
  camera.near = dist / 100;
  camera.far = dist * 100;
  camera.updateProjectionMatrix();
}

const VIEWS: Record<string, THREE.Vector3> = {
  iso: new THREE.Vector3(0.9, -1.2, 0.8),
  top: new THREE.Vector3(0, -0.0001, 1),
  front: new THREE.Vector3(0, -1, 0.0001),
  side: new THREE.Vector3(1, 0, 0.0001),
};

function setView(v: string) {
  frame(bounds(parts.map((p) => p.mesh)), VIEWS[v].clone().normalize());
}

function focus(p: Part) {
  frame(bounds([p.mesh]).expandByScalar(15));
}

// ---------- events ----------
document.querySelectorAll<HTMLButtonElement>("[data-view]").forEach((b) => (b.onclick = () => setView(b.dataset.view!)));
$("fit").onclick = () => (selected ? focus(selected) : setView("iso"));
$("explode").oninput = applyExplode;
$("section").onchange = applySection;
$("cut").oninput = () => {
  $<HTMLInputElement>("section").checked = true;
  applySection();
};
$("panelOpacity").oninput = applyPanelOpacity;
$("colorMode").onchange = recolor;
$("loadCase").onchange = () => {
  recolor();
  updateForces();
};
for (const id of ["showForces", "forceMount", "loadFrac", "arrowScale", "padForces"]) $(id).oninput = updateForces;
$("play").onclick = play;

let down = { x: 0, y: 0 };
renderer.domElement.addEventListener("pointerdown", (e) => (down = { x: e.clientX, y: e.clientY }));
renderer.domElement.addEventListener("pointerup", (e) => {
  if (Math.hypot(e.clientX - down.x, e.clientY - down.y) < 4 && e.button === 0) select(pick(e));
});
renderer.domElement.addEventListener("dblclick", (e) => {
  const p = pick(e);
  if (p) focus(p);
});
renderer.domElement.addEventListener("pointermove", (e) => {
  if (e.buttons) return;
  const p = pick(e);
  if (p !== hovered) {
    hovered = p;
    highlight();
    renderer.domElement.style.cursor = p ? "pointer" : "";
  }
});
window.addEventListener("keydown", (e) => {
  if (e.key === "Escape") select(undefined);
  if (e.key === "h" && selected) setVisible(selected, false);
  if (e.key === "a") parts.forEach((p) => setVisible(p, true));
  if (e.key === "f") $("fit").click();
});

function resize() {
  const { clientWidth: w, clientHeight: h } = viewport;
  renderer.setSize(w, h);
  labelRenderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport);
resize();

renderer.setAnimationLoop(() => {
  controls.update();
  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
});

load().catch((err) => {
  $("info").textContent = `Failed to load assets: ${err}. Run "npm run sync" after the FreeCAD build.`;
  $("info").classList.remove("hidden");
});
