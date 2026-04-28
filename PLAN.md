# Hydraulic Voltage — physics & visuals overhaul

## Context

The simulator currently has four problems:

1. **Tank heights blow up.** With pump off, water levels should equalise (communicating vessels). Instead the right side pegs to `H_MAX`, left to 0, and the two swap every couple of frames. Root cause: `Component.pipe` uses `k = 1e3` (`js-model.html:67`) so the bottom-loop time constant is ~0.6 ms, but the integrator at `js-solver.html:166-170` is forward-Euler with smallest sub-step ~2 ms — well past the `dt < 2τ` stability bound. The README/comments call it "semi-implicit" but it isn't.
2. **Pump is an ideal voltage source.** `PUMP.R_INT = 0.05` exists in CONFIG but is unused. Real pumps trade dP for Q (torque-vs-back-pressure). User wants a torque/power model: stalled = max dP, free-flow = max Q, linear between (Thévenin).
3. **Pump should sit below the tank** with zero-resistance plumbing right-tank → pump → left-tank, drawn as a single duct under the entire chamber row.
4. **Visuals are weak.** Barriers are flat hatched rectangles (should look porous, like sponges). Water is four flat axonometric quads with no surface motion or depth shading.

User confirmed: **linear Thévenin pump model**, **relocate pump fully under the tank** visually. Implement in stages so each stage can be tested and pushed independently via `clasp push`.

---

## Stage 1 — Make the integrator implicit (fixes the swap bug)

**Goal:** Single matrix solve per sub-step that includes tank dynamics. After this stage, with pump off, all chambers should monotonically converge to the same height.

Files: `js-solver.html`, `js-model.html`.

1. **`js-solver.html:22-32`** — stop pinning tanks as Dirichlet. Assign `solverIdx` to *every* node (tanks + junctions); drop the `tanks → solverIdx = -1` loop and the `t.p = t.h` initialiser.
2. **`js-solver.html:79-114`** — collapse the four-case stamp branch into a single `mnaStamp(G, ia, ib, k)` (both indices are now ≥ 0). Delete the tank-tank skip at line 113. Pump V-source stamp (lines 86-90) is unchanged.
3. **`js-solver.html`, new pass after edge stamping** — for each tank `t`:
   ```
   G[t.solverIdx][t.solverIdx] += t.A / dt;
   b[t.solverIdx]              += (t.A / dt) * t.h;
   ```
   This adds the capacitive term `(A/dt)·(h_new − h_old) = Q_net(h_new)`.
4. **`js-solver.html:42-43`** — after the solve, write `t.p = x[t.solverIdx]` for tanks too (so `computeEdgeFlows` sees consistent values inside the diode loop).
5. **`js-solver.html:152-171`** — gut `integrateHeights` down to:
   ```
   for (const t of tanks) {
     t.lastNetFlow = (t.A / dt) * (t.p - t.h);  // for renderer use later
     t.h = clamp(t.p, 0, CONFIG.TANK.H_MAX);
   }
   ```
   The matrix already integrated; this just commits and stashes net-flow for water visuals.
6. **`js-solver.html:139`** — delete the dead `c.comp` line (line 141 already overrides it).
7. **`js-model.html:67`** — drop `k: 1e3` to `k: 200`. Implicit integration handles either, but lower k is friendlier for debugging and matrix conditioning.

Diode loop interaction is fine: diodes still toggle k between `kFwd` / `kReverse` and re-solve; nothing in the iteration logic depended on tanks being Dirichlet.

Matrix size: ~13×13 worst case (8 slots → 9 chambers + 2-3 pump junctions + 1 pump current). `solveLinear` in `js-math.html:8` is dense Gauss; trivially fast.

**Verify Stage 1:**
- Pump = 0, all chambers initially at `H_INITIAL = 2.0`: heights stay at 2.0 forever (no drift, no flicker).
- Pump = 0, manually edit one tank's `h` to 4.0 in DevTools: all heights converge smoothly to ~2.something within a second.
- Pump = 1.0: steady state reached, no oscillation, dP across pump = `MAX_DP`.

---

## Stage 2 — Thévenin pump + relocated under-tank plumbing

**Goal:** Pump shows torque-vs-back-pressure tradeoff; plumbing visibly runs under the chamber row.

Files: `js-config.html`, `js-layout.html`, `js-render.html`, `js-model.html`.

### Solver/model changes

1. **`js-config.html:45-50`** — bump `R_INT` from `0.05` to `0.3` so no-load Q is in the same order as barrier flows (otherwise pump dominates everything). Add `PUMP.UNDER_GAP = 1.5` (extra Y drop below current `Y_BOT`).
2. **`js-layout.html:84-85`** — add a third pump junction `nPumpMid` between `nPumpIn` and `nPumpOut`.
3. **`js-layout.html:138-146`** — split the pump edge:
   - new `mkEdge` from `nPumpIn` → `nPumpMid` with `Component.resistor(1 / CONFIG.PUMP.R_INT)`, role `'pump-rint'`.
   - existing pump V-source edge becomes `nPumpMid` → `nPumpOut` (unchanged otherwise).
   The series-R + V-source pair is the standard Thévenin stamp using existing helpers.
4. **`js-model.html`** — no API change; just confirm `Component.pump(dP)` and `Component.resistor(k)` are reused.

### Visual relocation

5. **`js-layout.html:32`** — drop `Y_BOT` further (e.g. `-1.8 - PUMP.UNDER_GAP = -3.3`) so the pump and return ducts visibly sit under the tank with a clear gap. Update right/left drop-pipe paths in `js-layout.html:126-158` to follow the longer vertical span.
6. **`js-render.html:124-142` (`drawReturnPipe`)** — redraw as a single horizontal duct beneath the tank with two short vertical drop pipes at each end. Use existing `extrudePrism` (`js-project.html:49`, currently unused) to make the duct a real 3D box rather than the current ad-hoc face soup.
7. **`js-render.html:145-173` (`drawPump`)** — recentre the impeller at the new `Y_BOT + height/2`. Add an "internal resistance" visual cue: a small coil or zigzag drawn between `nPumpIn` and `nPumpMid` so the user can see the torque resistance.
8. **`js-render.html:271-279` (`schBattery`)** — show the source dP (matches the symbol's "ideal source" semantic). Optionally show `terminal dP = source dP − iPump · R_int` as a smaller secondary label.

**Verify Stage 2:**
- Slowly raise pump slider with no slots in carousel: dP_observed (probe across pump junctions) ≤ `MAX_DP`, and reduces as Q rises.
- Add a high-resistance slot (low k): dP across pump approaches `MAX_DP`, Q approaches 0.
- Add a low-resistance slot: Q approaches `MAX_DP / R_INT`, dP across pump approaches 0.
- The bottom duct is visually below the tank with a clear gap; particles still flow through it.

---

## Stage 3 — Sponge-textured barriers

**Goal:** Replace hatch lines with a procedural porous texture whose density tracks `k` (lower k = denser, less porous).

Files: `js-render.html`, `js-model.html`.

1. **`js-model.html`, `Slot` class** — add `_spongeCache = null` field.
2. **`js-render.html`, new helper `getSpongeTexture(slot, k, w, h)`** — returns a cached offscreen `<canvas>`:
   - Base fill: existing barrier colour.
   - Stamp `~round(60 * (1 - k/K_MAX) + 30)` random circles of radius 1.5–3 px in a slightly darker shade (creates the visible "holes" of a sponge).
   - Add a few highlight circles in lighter shade for surface texture.
   - Cache invalidates when `slot._spongeCache?.k !== k`.
3. **`js-render.html:88-121` (`drawOneBarrier`)** — replace the front-face fill at line 98 with `ctx.fillStyle = ctx.createPattern(getSpongeTexture(...), 'repeat')`. Delete the hatching block (lines 99-111). Keep the diode arrow (lines 113-120) and the back/side/top face polys (lines 93-95) — those still convey the 3D form.
4. Texture is regenerated only on slider drag; budget ~8k pixel writes, well under frame budget.

**Verify Stage 3:**
- Slide a resistor slot's k from min → max: the front face visibly goes from "dense sponge" to "open / porous". No frame hitch on drag.
- Schematic-mode toggle still draws the rectangle resistor symbol.

---

## Stage 4 — Water that looks like water

**Goal:** Surface waves that respond to flow; depth gradient on side faces; meniscus highlight; bubbles in active chambers.

Files: `js-render.html` only.

Replace `drawWater` (`js-render.html:50-69`) with a richer per-tank routine. For each tank:

1. **Subdivided wave surface (top face).** 24 segments along x. Each segment's y is `h + Σ amplitude · sin(k_i · x + ω_i · t + phase)` where amplitude scales with `|t.lastNetFlow|` (stashed in Stage 1). 3 sine components with different `k_i`, `ω_i`. Fill with the existing surface-blue, stroke crests with a lighter highlight. Cost: ~720 sine evals/frame total — trivial.
2. **Side faces with linear gradient.** Replace flat fill at lines 60, 62, 63, 67 with `ctx.createLinearGradient(p_top.sx, p_top.sy, p_bottom.sx, p_bottom.sy)` per face, going from `rgba(140,200,235,0.6)` near surface to `rgba(28,70,120,0.85)` at floor. Creates depth perception.
3. **Meniscus band.** Just below the wavy surface, draw a 2px-tall lighter-blue strip (one extra poly per tank).
4. **Local bubbles.** In each tank with `|lastNetFlow| > threshold`, render `n` small white circles at randomized (x, h·rand, z) with a short lifetime. State lives in a tank-local rolling array (don't touch the existing edge-particle pool — those are for pipe flow). Cheap.

Render order inside the tank loop: back face → side faces (gradient) → front face (gradient) → wavy surface mesh → meniscus → bubbles. Keep existing particle draw (`drawParticles`, line 197) for pipes.

**Verify Stage 4:**
- Pump on at 50%: visible flow-through chambers show wave activity proportional to flow; capacitor chamber (high A) shows slow gentle rise/fall.
- Pump off: surfaces are flat after Stage 1 settles them.
- Frame rate stays at 60 fps with 8 slots.

---

## Critical files

- `/home/user/hydraulic-voltage/js-solver.html` — Stage 1 (integration), minor in Stage 2.
- `/home/user/hydraulic-voltage/js-layout.html` — Stage 2 (extra junction, lowered Y_BOT, longer drop paths).
- `/home/user/hydraulic-voltage/js-model.html` — Stage 1 (`k: 200`), Stage 3 (`Slot._spongeCache`).
- `/home/user/hydraulic-voltage/js-render.html` — Stage 2 (relocated drawReturnPipe/drawPump), Stage 3 (drawBarriers), Stage 4 (drawWater).
- `/home/user/hydraulic-voltage/js-config.html` — Stage 2 (R_INT, UNDER_GAP).
- `/home/user/hydraulic-voltage/js-project.html` — read-only; reuse `extrudePrism` (line 49) in Stage 2.

## End-to-end verification

After all four stages, push to GAS via `clasp push && clasp deploy -d "vN"`. Open the `/exec` URL and exercise:

1. **Equilibrium** — pump off, default 1 slot: heights converge and stay equal.
2. **Pump curve** — sweep slider 0→1 with one resistor slot, watch ΔP probe across pump shrink with Q (Thévenin trade-off visible).
3. **Diode** — replace slot with diode, pump on: Q stays positive (forward), zero in reverse orientation.
4. **Capacitor** — replace slot with capacitor, pump step from 0 → 1: chamber fills with characteristic RC curve.
5. **Schematic mode** — toggle: overlay still aligns; battery label shows source dP.
6. **Visuals** — barriers look porous; water has waves under flow, settles flat at rest; pump duct visibly under the tank.

If Stage 1 alone fixes the swap bug and pump-off equilibrium works, the rest is purely additive — each subsequent stage can be reverted independently.
