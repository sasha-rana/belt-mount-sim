// Statics overlay: draws the MuJoCo pull-test forces (build/forces.json) as
// arrows on the assembly.
//
// Overview: the belt load, every interface's resultant (in a column beside the
// mount, with a leader to where it acts), bolt tension and the panel reaction.
// Free body: with a part selected, every force acting on that part, at its
// point of action, and the sum check.
//
// Convention: contact forces are drawn pushing into the part they act on (tip
// at the contact); the belt load, thread pull and panel reaction are drawn
// from their point of action.

import * as THREE from "three";
import { CSS2DObject } from "three/examples/jsm/renderers/CSS2DRenderer.js";

export type Vec3 = [number, number, number];
export type Body = "eyebolt" | "nut" | "spreader" | "backing" | "plug" | "panel";

export interface ForceItem {
  name: string;
  on: Body;
  from: Body;
  total: Vec3;
  point: Vec3 | null;
  pads: [Vec3, Vec3][];
  reaction?: boolean;        // the other side of a contact listed elsewhere
}
export interface MountFrame {
  applied: { point: Vec3; total: Vec3 };
  tension: number;
  shoulder_seated: boolean;
  forces: ForceItem[];
}
export interface Frame { fraction: number; mounts: Record<string, MountFrame> }
export interface ForceCase { description: string; load_N: number; tilt_deg: number; frames: Frame[] }
export interface ForcesFile { cases: Record<string, ForceCase> }

export interface ForceViewState {
  frame: Frame;
  mounts: string[];          // which mounts to draw
  mmPerKN: number;           // arrow scale
  pads: boolean;             // per-pad distribution instead of resultants
  body?: Body;               // free body of this part, or the overview
  explode: (body: Body) => number;   // z offset of a body in the exploded view
}

export const BODY_LABEL: Record<Body, string> = {
  eyebolt: "eyebolt", nut: "nut", spreader: "spreader", backing: "backing plate",
  plug: "panel plug (skins + bushing + annulus)", panel: "panel",
};

export const COLORS: Record<string, number> = {
  belt: 0xd62828, spreader: 0xf08c00, collar: 0x7b2cbf, wall: 0xd63384, nut: 0x0f9d8a,
  backing: 0x1c64f2, hole: 0x8d5524, thread: 0x2b9348, panel: 0x222222, "punch-through": 0x222222,
};

const CONTACT = new Set(["spreader", "collar", "wall", "nut", "backing", "hole"]);
const MIN_N = 5;               // forces below this are not drawn
const PAD_BOOST = 6;           // pad arrows are this much longer per kN than resultants

export function describe(f: ForceItem): string {
  if (f.name === "thread") return f.on === "nut" ? "bolt tension (thread pulls nut up)" : "bolt tension (nut pulls shank down)";
  if (f.name === "panel") return "panel holds plug";
  if (f.name === "punch-through") return "plug pushes on panel (punch-through)";
  return `${BODY_LABEL[f.from]} → ${BODY_LABEL[f.on]}`;
}

export const kN = (v: Vec3 | number[]) => Math.hypot(...v) / 1000;
export const fmt = (n: number) => `${n.toFixed(Math.abs(n) < 10 ? 2 : 1)} kN`;

export class ForceView {
  readonly group = new THREE.Group();

  update(s: ForceViewState) {
    this.clear();
    for (const m of s.mounts) {
      const mf = s.frame.mounts[m];
      if (!mf) continue;
      if (s.body) this.freeBody(mf, s);
      else this.overview(m, mf, s);
    }
  }

  private overview(mount: string, mf: MountFrame, s: ForceViewState) {
    const ax = mf.applied.point;
    this.arrow(ax, mf.applied.total, "belt", s.mmPerKN, "tail", `belt ${fmt(kN(mf.applied.total))}`);
    // Resultant columns sit outboard of the mount so collinear forces don't overlap
    const side = ax[0] < 0 ? -1 : 1;
    let col = 0;
    for (const f of mf.forces) {
      if (!CONTACT.has(f.name) || f.reaction || !f.point || kN(f.total) * 1000 < MIN_N) continue;
      const z = s.explode(f.on);
      const at: Vec3 = [f.point[0], f.point[1], f.point[2] + z];
      if (s.pads) {
        this.padArrows(f, s.mmPerKN, z);
        this.label(at, `${f.name} ${fmt(kN(f.total))}`, f.name, [side * 34, 0, 0]);
      } else {
        const callout: Vec3 = [ax[0] + side * (36 + 26 * col++), at[1], at[2]];
        this.leader(at, callout, f.name);
        this.arrow(callout, f.total, f.name, s.mmPerKN, "tip", `${f.name} ${fmt(kN(f.total))}`);
      }
    }
    // Bolt tension as an opposed pair on the shank, and the panel reaction at the plug
    const nutF = mf.forces.find((f) => f.name === "thread" && f.on === "nut");
    if (nutF?.point && mf.tension > MIN_N) {
      const p: Vec3 = [nutF.point[0], nutF.point[1], nutF.point[2] + s.explode("nut")];
      this.arrow(p, [0, 0, mf.tension], "thread", s.mmPerKN * 0.5, "tail", `bolt tension ${fmt(mf.tension / 1000)}`);
    }
    const punch = mf.forces.find((f) => f.name === "punch-through");
    if (punch?.point && kN(punch.total) * 1000 > MIN_N) {
      const p: Vec3 = [punch.point[0] - side * 32, punch.point[1], punch.point[2]];
      this.leader(punch.point, p, "panel");
      this.arrow(p, punch.total, "punch-through", s.mmPerKN, "tail", `punch-through ${fmt(kN(punch.total))}`);
    }
    void mount;
  }

  private freeBody(mf: MountFrame, s: ForceViewState) {
    const body = s.body!;
    const z = s.explode(body);
    if (body === "eyebolt") {
      const p = mf.applied.point;
      this.arrow([p[0], p[1], p[2] + z], mf.applied.total, "belt", s.mmPerKN, "tail", `belt ${fmt(kN(mf.applied.total))}`);
    }
    for (const f of mf.forces) {
      if (f.on !== body || !f.point || kN(f.total) * 1000 < MIN_N) continue;
      const at: Vec3 = [f.point[0], f.point[1], f.point[2] + z];
      if (s.pads && f.pads.length) {
        this.padArrows(f, s.mmPerKN, z);
        this.label(at, `${f.name} ${fmt(kN(f.total))}`, f.name, [0, -30, 0]);
      } else {
        this.arrow(at, f.total, f.name, s.mmPerKN, CONTACT.has(f.name) ? "tip" : "tail", `${f.name} ${fmt(kN(f.total))}`);
      }
    }
  }

  private padArrows(f: ForceItem, mmPerKN: number, z: number) {
    for (const [p, v] of f.pads) {
      if (Math.hypot(...v) < 1) continue;
      this.arrow([p[0], p[1], p[2] + z], v, f.name, mmPerKN * PAD_BOOST, "tip", undefined, true);
    }
  }

  private arrow(at: Vec3, f: Vec3, kind: string, mmPerKN: number, anchor: "tip" | "tail", text?: string, thin = false) {
    const v = new THREE.Vector3(...f);
    const len = Math.max((v.length() / 1000) * mmPerKN, 1.5);
    const dir = v.clone().normalize();
    const p = new THREE.Vector3(...at);
    const origin = anchor === "tip" ? p.clone().addScaledVector(dir, -len) : p;
    this.group.add(solidArrow(origin, dir, len, COLORS[kind] ?? 0x444444, thin ? 0.3 : 0.75));
    if (text) {
      const end = origin.clone().addScaledVector(dir, anchor === "tip" ? 0 : len);
      this.label(end.toArray() as Vec3, text, kind);
    }
  }

  private leader(a: Vec3, b: Vec3, kind: string) {
    const g = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...a), new THREE.Vector3(...b)]);
    const m = new THREE.LineDashedMaterial({ color: COLORS[kind] ?? 0x444444, dashSize: 1.5, gapSize: 1.2, depthTest: false });
    const line = new THREE.Line(g, m);
    line.computeLineDistances();
    line.renderOrder = 9;
    this.group.add(line);
  }

  private label(at: Vec3, text: string, kind: string, offset: Vec3 = [0, 0, 0]) {
    const el = document.createElement("div");
    el.className = "force-label";
    el.textContent = text;
    el.style.borderColor = `#${new THREE.Color(COLORS[kind] ?? 0x444444).getHexString()}`;
    const obj = new CSS2DObject(el);
    obj.position.set(at[0] + offset[0], at[1] + offset[1], at[2] + offset[2]);
    this.group.add(obj);
  }

  private clear() {
    for (const c of [...this.group.children]) {
      this.group.remove(c);
      if (c instanceof CSS2DObject) c.element.remove();
      c.traverse((o) => {
        const m = o as THREE.Mesh;
        m.geometry?.dispose();
        (Array.isArray(m.material) ? m.material : [m.material]).forEach((x) => x?.dispose());
      });
    }
  }
}

/** Rows for the side table: the overview summary or a part's free body. */
export function table(mf: MountFrame, body?: Body): { label: string; value: string; color?: number }[] {
  if (body) {
    const rows: { label: string; value: string; color?: number }[] = [];
    const sum = new THREE.Vector3();
    if (body === "eyebolt") {
      rows.push({ label: "belt load", value: vecText(mf.applied.total), color: COLORS.belt });
      sum.add(new THREE.Vector3(...mf.applied.total));
    }
    for (const f of mf.forces.filter((f) => f.on === body && kN(f.total) * 1000 >= MIN_N)) {
      rows.push({ label: describe(f), value: vecText(f.total), color: COLORS[f.name] });
      sum.add(new THREE.Vector3(...f.total));
    }
    // Quasi-static: a residual of more than ~1% of the belt load means the step hadn't settled
    rows.push({ label: "Σ F (≈ 0 in equilibrium)", value: vecText(sum.toArray() as Vec3) });
    return rows;
  }
  const get = (n: string) => mf.forces.find((f) => f.name === n && !f.reaction);
  const rows = [
    { label: "belt load", value: fmt(kN(mf.applied.total)), color: COLORS.belt },
    { label: "bolt tension", value: fmt(mf.tension / 1000), color: COLORS.thread },
    { label: "shoulder on spreader", value: mf.shoulder_seated ? "seated (clamped)" : "lifted off (gapped)" },
  ];
  for (const n of ["collar", "wall", "spreader", "nut", "backing", "hole"]) {
    const f = get(n);
    if (f) rows.push({ label: describe(f), value: fmt(kN(f.total)), color: COLORS[n] });
  }
  const p = get("punch-through");
  if (p) rows.push({ label: "punch-through on panel", value: fmt(kN(p.total)), color: COLORS.panel });
  return rows;
}

function vecText(v: Vec3) {
  return `${fmt(kN(v))}  (${v.map((x) => (x / 1000).toFixed(2)).join(", ")})`;
}

/** Arrow with a cylinder shaft and cone head, drawn on top of the parts. */
function solidArrow(origin: THREE.Vector3, dir: THREE.Vector3, len: number, color: number, radius: number) {
  const g = new THREE.Group();
  const mat = new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true });
  const head = Math.min(len * 0.35, radius * 6);
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, len - head, 10), mat);
  shaft.position.y = (len - head) / 2;
  const cone = new THREE.Mesh(new THREE.ConeGeometry(radius * 2.6, head, 14), mat);
  cone.position.y = len - head / 2;
  g.add(shaft, cone);
  g.position.copy(origin);
  g.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
  g.renderOrder = 10;
  shaft.renderOrder = cone.renderOrder = 10;
  return g;
}
