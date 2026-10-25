/* Speedo — accelerometer + GPS speedometer with a trip odometer.
 *
 * How speed is estimated
 * ----------------------
 * A tiny 2-state Kalman filter tracks [speed, accelerometer bias].
 *  - Every motion sample (~60 Hz) predicts: speed += (forward accel - bias) * dt.
 *  - Every GPS fix (~1 Hz) corrects speed, and over time learns the bias.
 *  - "Forward" is an axis in the phone's own frame. It's learned by matching the
 *    accelerometer against GPS speed changes, or from the first pull-away from a stop,
 *    and guessed from the phone's tilt until then. Re-mount the phone and it relearns.
 * Accelerometer alone drifts (integrating noise always does), so with no GPS it's
 * dead reckoning: snappy, but expect it to wander over minutes.
 */
'use strict';
(() => {
  const $ = (s) => document.querySelector(s);
  const SVGNS = 'http://www.w3.org/2000/svg';
  const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
  const now = () => performance.now() / 1000;

  // ---------- small vector kit (device frame, m/s²) ----------
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const mul = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
  const mag = (a) => Math.hypot(a[0], a[1], a[2]);
  const norm = (a) => { const l = mag(a); return l > 1e-9 ? mul(a, 1 / l) : [0, 0, 0]; };
  const horiz = (v, g) => { if (!g) return v; const n = norm(g); return sub(v, mul(n, dot(v, n))); };
  const angle = (a, b) => Math.acos(clamp(dot(norm(a), norm(b)), -1, 1));

  // ---------- persistence ----------
  const store = {
    get(k, d) { try { const v = localStorage.getItem(k); return v == null ? d : JSON.parse(v); } catch { return d; } },
    set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { /* private mode */ } },
  };
  const DEFAULTS = {
    view: 'gauge', units: 'mph', theme: 'phosphor', accent: '', gaugeMax: 'auto',
    source: 'auto', odoStyle: 'wheels', odoDigits: 5, odoDecimals: 1, mirror: false, wake: true,
  };
  const S = Object.assign({}, DEFAULTS, store.get('speedo.settings', {}));
  const saveSettings = () => store.set('speedo.settings', S);

  const UNITS = {
    mph: { label: 'mph',  k: 2.2369363, dist: 'mi',  dk: 1 / 1609.344, max: 120, dp: 0 },
    kmh: { label: 'km/h', k: 3.6,       dist: 'km',  dk: 1 / 1000,     max: 180, dp: 0 },
    ms:  { label: 'm/s',  k: 1,         dist: 'km',  dk: 1 / 1000,     max: 50,  dp: 1 },
    kn:  { label: 'kn',   k: 1.9438445, dist: 'nmi', dk: 1 / 1852,     max: 100, dp: 0 },
  };

  // ---------- sensor state ----------
  const STILL_RMS = 0.07;          // m/s² — below this the phone is resting, not riding
  const E = {
    s: 0, b: 0, P00: 1, P01: 0, P11: 0.04,   // Kalman: speed (m/s), bias (m/s²), covariance
    fwd: null, fwdConf: 0, fwdFrom: '', gRef: null,
    g: null, linMean: [0, 0, 0], vib: 1, stillT: 0, still: false, lastStillT: -99, zeroT: 1,
    motionOK: false, lastMotionT: 0, hz: 0, hzN: 0, hzT0: 0,
    gpsOK: false, gpsErr: 0, gpsWeak: false, gpsAcc: null, gpsSpeed: null, gpsDerived: false,
    lastFixT: 0, lastFix: null, fixes: [], resumedAt: 0,
    sumLin: [0, 0, 0], nLin: 0,
    launch: { sum: [0, 0, 0], magSum: 0, n: 0, t: 0 },
  };

  // ---------- Kalman filter ----------
  function predict(u, dt, useAccel) {
    const f = useAccel ? -dt : 0;
    if (useAccel) E.s += (u - E.b) * dt;
    const qs = (useAccel ? 0.12 : 2.25) * dt;   // speed can change faster when we're blind to accel
    const qb = 4e-4 * dt;
    const P00 = E.P00 + 2 * f * E.P01 + f * f * E.P11 + qs;
    const P01 = E.P01 + f * E.P11;
    E.P00 = Math.min(P00, 400); E.P01 = P01; E.P11 = Math.min(E.P11 + qb, 1);
    if (E.s < 0) E.s = 0;
  }
  function correct(z, R) {
    const Sx = E.P00 + R, k0 = E.P00 / Sx, k1 = E.P01 / Sx, y = z - E.s;
    E.s = Math.max(0, E.s + k0 * y);
    E.b = clamp(E.b + k1 * y, -1.5, 1.5);
    const P00 = (1 - k0) * E.P00, P01 = (1 - k0) * E.P01, P11 = E.P11 - k1 * E.P01;
    E.P00 = P00; E.P01 = P01; E.P11 = Math.max(P11, 1e-6);
  }

  // Before anything is learned: phone upright in a mount → the back of the phone faces
  // the road ahead. Phone lying flat → its top edge does.
  function guessForward(g) {
    if (!g) return [0, 0, -1];
    const back = horiz([0, 0, -1], g);
    if (mag(back) > 0.5) return norm(back);
    return norm(horiz([0, 1, 0], g));
  }

  // ---------- accelerometer ----------
  function onMotion(e) {
    const t = now();
    const ag = e.accelerationIncludingGravity, a = e.acceleration;
    const hasAG = !!ag && ag.x != null, hasA = !!a && a.x != null;
    if (!hasAG && !hasA) return;
    const dt = E.lastMotionT ? t - E.lastMotionT : 0;
    E.lastMotionT = t;
    E.motionOK = true;
    E.hzN++;
    if (t - E.hzT0 >= 1) { E.hz = E.hzN / (t - E.hzT0); E.hzN = 0; E.hzT0 = t; }

    const A = hasA ? [a.x, a.y, a.z] : null;
    const AG = hasAG ? [ag.x, ag.y, ag.z] : null;
    const fresh = dt > 0 && dt < 0.25;
    // Gravity is exact when the OS hands us both vectors; otherwise slow low-pass.
    const gRaw = A && AG ? sub(AG, A) : AG;
    if (gRaw) {
      const k = E.g && fresh ? 1 - Math.exp(-dt / (A ? 0.25 : 3)) : 1;
      E.g = E.g ? add(E.g, mul(sub(gRaw, E.g), k)) : gRaw;
    }
    if (!fresh) return;   // first sample, or we just came back from the background
    const lin = A || sub(AG, E.g);

    // Vibration level → "is the phone resting?"
    E.linMean = add(E.linMean, mul(sub(lin, E.linMean), 1 - Math.exp(-dt / 1)));
    const dev = sub(lin, E.linMean);
    E.vib += (dot(dev, dev) - E.vib) * (1 - Math.exp(-dt / 0.5));
    E.stillT = Math.sqrt(E.vib) < STILL_RMS ? E.stillT + dt : 0;
    E.still = E.stillT > 1.2;
    if (E.still) E.lastStillT = t;

    // Phone rotated > ~30° since we learned "forward"? It was re-mounted; relearn.
    if (E.fwd && E.gRef && E.g && angle(E.g, E.gRef) > 0.5) { E.fwd = null; E.fwdConf = 0; E.fwdFrom = ''; }

    E.sumLin = add(E.sumLin, lin); E.nLin++;
    const linH = horiz(lin, E.g);
    if (E.fwdConf < 1) detectLaunch(linH, dt, t);

    if (S.source === 'gps' || demo) return;
    const gpsFresh = E.gpsOK && t - E.lastFixT < 3;
    const useAccel = S.source === 'accel' || !gpsFresh || !!E.fwd;
    const fwd = E.fwd || guessForward(E.g);
    predict(useAccel ? dot(lin, fwd) : 0, dt, useAccel);
    if (E.still && !gpsFresh) correct(0, 0.01);   // zero-velocity update while resting
    E.zeroT = E.s < 0.3 ? E.zeroT + dt : 0;
  }

  // The first firm, steady shove after sitting still is almost always "forward".
  function detectLaunch(h, dt, t) {
    const L = E.launch;
    // Must start from a real stop (speed ~0 for a moment), not the tail end of braking.
    const atRest = E.zeroT > 0.8 || t - E.lastStillT < 3;
    const m = mag(h);
    if (m > 0.8 && (L.n > 0 || atRest)) { L.sum = add(L.sum, h); L.magSum += m; L.n++; L.t += dt; }
    else { L.sum = [0, 0, 0]; L.magSum = 0; L.n = 0; L.t = 0; return; }
    if (L.t < 0.4) return;
    const avg = mul(L.sum, 1 / L.n);
    if (mag(avg) > 0.75 * (L.magSum / L.n)) {        // direction held steady, not a shake
      E.fwd = norm(avg); E.fwdConf = 0.5; E.fwdFrom = 'launch'; E.gRef = E.g && E.g.slice();
      E.s = Math.max(E.s, mag(avg) * L.t);
    }
    L.sum = [0, 0, 0]; L.magSum = 0; L.n = 0; L.t = 0;
  }

  // Match the average acceleration between two fixes to GPS's own change in speed.
  function learnAxis(gpsAccel) {
    const n = E.nLin, sum = E.sumLin;
    E.sumLin = [0, 0, 0]; E.nLin = 0;
    if (gpsAccel == null || n < 5 || !E.g || Math.abs(gpsAccel) < 0.5) return;
    const h = horiz(mul(sum, 1 / n), E.g), m = mag(h), ratio = m / Math.abs(gpsAccel);
    if (m < 0.3 || ratio < 0.4 || ratio > 2.5) return;   // mostly cornering, or noise
    const cand = mul(h, Math.sign(gpsAccel) / m);
    if (!E.fwd || E.fwdConf < 1 || (dot(E.fwd, cand) < 0 && E.fwdConf < 3)) E.fwd = cand;
    else if (dot(E.fwd, cand) < 0) { E.fwdConf -= 1; return; }
    else E.fwd = norm(add(mul(E.fwd, 0.75), mul(cand, 0.25)));
    E.fwdConf = Math.min(Math.max(E.fwdConf, 0) + 1, 20); E.fwdFrom = 'gps'; E.gRef = E.g.slice();
  }

  // ---------- GPS ----------
  let watchId = null;
  function startGPS() {
    if (watchId != null || !('geolocation' in navigator)) return;
    watchId = navigator.geolocation.watchPosition(onPos, (err) => { E.gpsErr = err.code; },
      { enableHighAccuracy: true, maximumAge: 0 });
  }
  function stopGPS() {
    if (watchId != null) navigator.geolocation.clearWatch(watchId);
    watchId = null; E.gpsOK = false;
  }
  function haversine(a, b) {
    const R = 6371008.8, r = Math.PI / 180;
    const dLat = (b.lat - a.lat) * r, dLon = (b.lon - a.lon) * r;
    const h = Math.sin(dLat / 2) ** 2 + Math.cos(a.lat * r) * Math.cos(b.lat * r) * Math.sin(dLon / 2) ** 2;
    return 2 * R * Math.asin(Math.min(1, Math.sqrt(h)));
  }
  function speedFromFixes() {
    const f = E.fixes;
    if (f.length < 2) return null;
    const a = f[0], b = f[f.length - 1], dt = b.t - a.t;
    if (dt < 0.8) return null;
    const d = haversine(a, b);
    return d < Math.max(a.acc, b.acc) * 0.5 ? 0 : d / dt;   // inside the error circle → parked
  }
  function onPos(p) {
    const t = now(), c = p.coords;
    const fix = { t, lat: c.latitude, lon: c.longitude, acc: c.accuracy || 999 };
    E.gpsErr = 0; E.gpsAcc = fix.acc;
    // Nothing integrates while the page is hidden, so bridge that gap with straight-line distance.
    if (E.lastFix && !demo && (document.hidden || (E.resumedAt && E.lastFix.t < E.resumedAt))) {
      const d = haversine(E.lastFix, fix);
      if (d > E.lastFix.acc + fix.acc && d / (t - E.lastFix.t) < 90) addTrip(d);
      if (!document.hidden) E.resumedAt = 0;
    }
    E.lastFix = fix;
    E.fixes.push(fix);
    while (E.fixes.length > 2 && t - E.fixes[0].t > 4) E.fixes.shift();
    E.gpsWeak = fix.acc > 80;
    if (E.gpsWeak) return;   // Wi-Fi/cell-tower guesses: too vague for speed

    const native = Number.isFinite(c.speed) && c.speed >= 0;
    const sp = native ? c.speed : speedFromFixes();
    if (sp == null) return;
    const ga = E.gpsSpeed != null && t - E.lastFixT < 3 ? (sp - E.gpsSpeed) / Math.max(t - E.lastFixT, 0.2) : null;
    learnAxis(ga);
    E.gpsSpeed = sp; E.gpsDerived = !native; E.lastFixT = t; E.gpsOK = true;
    if (S.source === 'accel' || demo) return;
    const sd = (native ? 0.35 : 0.8) + fix.acc / 40;
    correct(sp, sd * sd);
  }

  // ---------- motion permission & lifecycle ----------
  // iOS (and newer Chrome) gate motion data behind requestPermission(), and iOS only
  // shows the prompt from inside a tap. So: ask quietly at boot; if that isn't enough
  // and no motion data shows up, put a Start button in front.
  const needsMotionPermission = typeof DeviceMotionEvent !== 'undefined' &&
    typeof DeviceMotionEvent.requestPermission === 'function';
  let motionPerm = needsMotionPermission ? 'unknown' : 'granted';
  let motionOn = false, started = false;

  function sensors() {
    if (!started) return;
    const wantMotion = S.source !== 'gps' && 'DeviceMotionEvent' in window;
    if (wantMotion && !motionOn) { window.addEventListener('devicemotion', onMotion); motionOn = true; }
    if (!wantMotion && motionOn) { window.removeEventListener('devicemotion', onMotion); motionOn = false; E.motionOK = false; }
    if (S.source === 'accel') stopGPS(); else startGPS();
  }

  async function askMotion() {   // call straight from a tap handler
    if (!needsMotionPermission || motionPerm === 'granted') return;
    try { motionPerm = await DeviceMotionEvent.requestPermission(); } catch { motionPerm = 'denied'; }
  }

  async function boot() {
    started = true;
    sensors();
    wake();
    if (!needsMotionPermission || S.source === 'gps') return;
    try { motionPerm = await DeviceMotionEvent.requestPermission(); } catch { motionPerm = 'needs-tap'; }
    if (motionPerm === 'granted') return;
    setTimeout(() => {
      if (E.motionOK || S.source === 'gps') return;
      const overlay = $('#start');
      overlay.hidden = false;
      $('#startBtn').addEventListener('click', async () => {
        overlay.hidden = true;
        await askMotion();
        wake();
      }, { once: true });
    }, 700);
  }

  // ---------- screen wake lock ----------
  let wakeLock = null;
  async function wake() {
    if (!S.wake) { if (wakeLock) { wakeLock.release().catch(() => {}); wakeLock = null; } return; }
    if (wakeLock || !('wakeLock' in navigator) || document.hidden) return;
    try {
      wakeLock = await navigator.wakeLock.request('screen');
      wakeLock.addEventListener('release', () => { wakeLock = null; });
    } catch { /* battery saver, or no user gesture yet */ }
  }

  // ---------- trip ----------
  let trip = Number(store.get('speedo.trip', 0)) || 0;
  let tripDirty = false, resetAnim = null;
  const addTrip = (m) => { trip += m; tripDirty = true; };
  const saveTrip = () => { if (tripDirty) { store.set('speedo.trip', trip); tripDirty = false; } };
  setInterval(saveTrip, 3000);

  function resetTrip() {
    resetAnim = { from: trip * UNITS[S.units].dk, t0: now() };
    trip = 0; tripDirty = true; saveTrip();
    if (navigator.vibrate) navigator.vibrate(25);
  }

  let hiddenAt = 0;
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { hiddenAt = now(); saveTrip(); return; }
    if (hiddenAt && now() - hiddenAt > 2) E.resumedAt = now();
    hiddenAt = 0;
    wake();
  });
  window.addEventListener('pagehide', saveTrip);

  // ---------- demo drive (for trying it at a desk) ----------
  let demo = false, demoT0 = 0;
  const smooth = (x) => x * x * (3 - 2 * x);
  function demoSpeed(t) {
    const c = (t - demoT0) % 40;
    if (c < 9) return 29 * smooth(c / 9);                       // pull away to ~65 mph
    if (c < 21) return 29 + 1.6 * Math.sin((c - 9) * Math.PI / 6);
    if (c < 30) return 29 * (1 - smooth((c - 21) / 9));          // brake to a stop
    return 0;
  }

  // ---------- seven-segment digits ----------
  const SEGS = (() => {
    const L = 5, Rr = 41, T = 5, M = 40, B = 75, h = 4.5, g = 1.3;
    const H = (x1, x2, y) => `${x1},${y} ${x1 + h},${y - h} ${x2 - h},${y - h} ${x2},${y} ${x2 - h},${y + h} ${x1 + h},${y + h}`;
    const V = (x, y1, y2) => `${x},${y1} ${x + h},${y1 + h} ${x + h},${y2 - h} ${x},${y2} ${x - h},${y2 - h} ${x - h},${y1 + h}`;
    return {
      a: H(L + g, Rr - g, T), b: V(Rr, T + g, M - g), c: V(Rr, M + g, B - g), d: H(L + g, Rr - g, B),
      e: V(L, M + g, B - g), f: V(L, T + g, M - g), g: H(L + g, Rr - g, M),
    };
  })();
  const GLYPH = { 0: 'abcdef', 1: 'bc', 2: 'abdeg', 3: 'abcdg', 4: 'bcfg', 5: 'acdfg', 6: 'acdefg', 7: 'abc', 8: 'abcdefg', 9: 'abcdfg', '-': 'g', ' ': '' };
  function segDigit() {
    const svg = document.createElementNS(SVGNS, 'svg');
    svg.setAttribute('viewBox', '0 0 56 81');
    svg.setAttribute('class', 'sd');
    svg.setAttribute('aria-hidden', 'true');
    const grp = document.createElementNS(SVGNS, 'g');
    grp.setAttribute('transform', 'translate(9 .5) skewX(-6)');
    svg._segs = Object.entries(SEGS).map(([k, pts]) => {
      const p = document.createElementNS(SVGNS, 'polygon');
      p.setAttribute('points', pts); p.dataset.s = k; grp.append(p); return p;
    });
    const dp = document.createElementNS(SVGNS, 'circle');
    dp.setAttribute('cx', '50'); dp.setAttribute('cy', '75'); dp.setAttribute('r', '4.2'); dp.setAttribute('class', 'dpdot');
    grp.append(dp); svg.append(grp);
    svg._ch = null;
    return svg;
  }
  function setSeg(svg, ch) {
    if (svg._ch === ch) return;
    svg._ch = ch;
    const on = GLYPH[ch] ?? '';
    for (const p of svg._segs) p.classList.toggle('on', on.includes(p.dataset.s));
  }

  // ---------- gauge ----------
  const gauge = $('#gauge');
  let gaugeMax = 120, arcEl = null, ptrEl = null;
  const A0 = 135, SWEEP = 270, R = 86;
  const polar = (deg, r) => { const a = deg * Math.PI / 180; return [r * Math.cos(a), r * Math.sin(a)]; };
  const fmt = (n) => +n.toFixed(2);
  function svgEl(tag, attrs) {
    const el = document.createElementNS(SVGNS, tag);
    for (const k in attrs) el.setAttribute(k, attrs[k]);
    gauge.append(el); return el;
  }
  function buildGauge() {
    const u = UNITS[S.units];
    gaugeMax = S.gaugeMax === 'auto' ? u.max : Number(S.gaugeMax);
    const step = [5, 10, 20, 25, 30, 40, 50, 100].find((s) => gaugeMax / s <= 10) || 100;
    gauge.replaceChildren();
    const [sx, sy] = polar(A0, R), [ex, ey] = polar(A0 + SWEEP, R);
    const d = `M${fmt(sx)} ${fmt(sy)} A${R} ${R} 0 1 1 ${fmt(ex)} ${fmt(ey)}`;
    svgEl('path', { d, class: 'track' });
    arcEl = svgEl('path', { d, class: 'arc', pathLength: 100, 'stroke-dasharray': '0 100' });
    const half = step / 2, count = Math.round(gaugeMax / half);
    for (let i = 0; i <= count; i++) {
      const v = i * half, major = i % 2 === 0, deg = A0 + (v / gaugeMax) * SWEEP;
      const [x1, y1] = polar(deg, 77), [x2, y2] = polar(deg, major ? 68 : 72);
      svgEl('line', { x1: fmt(x1), y1: fmt(y1), x2: fmt(x2), y2: fmt(y2), class: major ? 'tick major' : 'tick' });
      if (major) {
        const [lx, ly] = polar(deg, 57);
        svgEl('text', { x: fmt(lx), y: fmt(ly), class: 'lbl' }).textContent = v;
      }
    }
    ptrEl = svgEl('line', { x1: 77, y1: 0, x2: 95, y2: 0, class: 'ptr', transform: `rotate(${A0})` });
    lastFrac = -1;
  }

  // ---------- odometer ----------
  const odo = $('#odo');
  let odoCells = [], odoKey = '';
  function buildOdo() {
    const D = S.odoDigits, P = S.odoDecimals;
    odo.className = 'odo ' + S.odoStyle;
    odo.replaceChildren();
    odoCells = [];
    for (let i = 0; i < D + P; i++) {
      const isDec = i >= D;
      if (S.odoStyle === 'wheels') {
        const w = document.createElement('div');
        w.className = 'wheel' + (isDec ? ' dec' : '');
        const strip = document.createElement('div');
        strip.className = 'strip';
        for (let k = 0; k <= 10; k++) { const s = document.createElement('span'); s.textContent = k % 10; strip.append(s); }
        w.append(strip); odo.append(w);
        strip._pos = -1; odoCells.push(strip);
      } else {
        const d = segDigit();
        if (isDec) d.classList.add('dec');
        if (P && i === D - 1) d.classList.add('dp');
        odo.append(d); odoCells.push(d);
      }
    }
    odoKey = '';
  }
  function renderOdo(value) {
    const D = S.odoDigits, P = S.odoDecimals, N = D + P;
    const n = Math.max(0, (value * 10 ** P) % 10 ** N);
    if (S.odoStyle === 'wheels') {
      // Mechanical carry: a wheel only turns while every wheel to its right is rolling 9 → 0.
      for (let j = 0; j < N; j++) {
        const strip = odoCells[N - 1 - j], place = 10 ** j, raw = n / place;
        let pos;
        if (j === 0) pos = raw % 10;
        else {
          const lower = (n / (place / 10)) % 10;
          pos = (Math.floor(raw) % 10) + Math.max(0, lower - 9);
        }
        if (Math.abs(pos - strip._pos) > 1e-4) {
          strip._pos = pos;
          strip.style.transform = `translateY(${(-pos * 100 / 11).toFixed(3)}%)`;
        }
      }
    } else {
      const s = String(Math.floor(n + 1e-9)).padStart(N, '0');
      if (s === odoKey) return;
      odoKey = s;
      const firstShown = Math.min(s.search(/[1-9]/) === -1 ? N : s.search(/[1-9]/), D - 1);   // blank leading zeros
      odoCells.forEach((cell, i) => setSeg(cell, i < firstShown ? ' ' : s[i]));
    }
  }

  // ---------- LED readout ----------
  const led = $('#led');
  let ledCells = [];
  function buildLED() {
    led.replaceChildren();
    ledCells = [0, 1, 2].map(() => { const d = segDigit(); led.append(d); return d; });
  }
  function renderLED(val, dp) {
    const s = dp ? Math.min(val, 99.9).toFixed(1).replace('.', '') : String(Math.min(Math.round(val), 999));
    const chars = s.padStart(3, ' ');
    ledCells.forEach((c, i) => { setSeg(c, chars[i]); c.classList.toggle('dp', !!dp && i === 1); });
  }

  // ---------- frame loop ----------
  const numEl = $('#num'), readout = $('#readout'), statusEl = $('#status'), statusText = $('#statusText');
  let lastT = now(), shown = 0, moving = false, lastTxt = null, lastFrac = -1, lastStatus = '', lastStale = null;

  function statusFor(t, gpsFresh, motionFresh) {
    const acc = E.gpsAcc != null ? Math.round(E.gpsAcc) : null;
    if (demo) return ['good', 'Demo drive'];
    if (!window.isSecureContext) return ['bad', 'Needs HTTPS for sensors'];
    const blocked = E.gpsErr === 1;
    if (S.source === 'gps') {
      if (blocked) return ['bad', 'Location blocked'];
      if (gpsFresh) return ['good', `GPS ±${acc} m`];
      return E.gpsWeak ? ['approx', `Weak GPS ±${acc} m`] : ['approx', 'Finding GPS'];
    }
    if (S.source === 'accel') return motionFresh ? ['approx', 'Accelerometer only'] : ['bad', 'No motion sensor'];
    if (gpsFresh && motionFresh) return ['good', `GPS ±${acc} m + motion`];
    if (gpsFresh) return ['good', `GPS ±${acc} m`];
    if (motionFresh) return ['approx', blocked ? 'Motion only, location blocked' : 'Motion only, finding GPS'];
    return ['bad', blocked ? 'Location blocked' : 'Waiting for sensors'];
  }

  function frame() {
    const t = now(), dt = Math.min(t - lastT, 0.25);
    lastT = t;
    const motionFresh = E.motionOK && t - E.lastMotionT < 0.5;
    const gpsFresh = E.gpsOK && t - E.lastFixT < 4;

    if (demo) { E.s = demoSpeed(t); }
    else if (S.source === 'gps' || !motionFresh) predict(0, dt, false);

    const live = demo || (S.source === 'gps' ? gpsFresh : S.source === 'accel' ? motionFresh : gpsFresh || motionFresh);
    if (moving ? E.s < 0.3 : E.s > 0.6) moving = !moving;   // hysteresis kills GPS jitter at a stop
    const v = moving ? E.s : 0;
    if (live) addTrip(v * dt);
    shown += (v - shown) * (1 - Math.exp(-dt * 7));
    if (Math.abs(shown - v) < 0.003) shown = v;

    // speed
    const u = UNITS[S.units], val = shown * u.k;
    const txt = u.dp ? val.toFixed(u.dp) : String(Math.round(val));
    if (txt !== lastTxt) {
      lastTxt = txt;
      if (S.view === 'led') renderLED(val, u.dp);
      else { numEl.textContent = txt; numEl.style.setProperty('--chars', txt.length); }
    }
    if (S.view === 'gauge') {
      const frac = clamp(val / gaugeMax, 0, 1);
      if (Math.abs(frac - lastFrac) > 4e-4) {
        lastFrac = frac;
        arcEl.setAttribute('stroke-dasharray', `${(frac * 100).toFixed(2)} 100`);
        arcEl.style.opacity = frac < 0.003 ? 0 : 1;
        ptrEl.setAttribute('transform', `rotate(${(A0 + frac * SWEEP).toFixed(2)})`);
      }
    }
    const stale = !live && started;
    if (stale !== lastStale) { lastStale = stale; readout.classList.toggle('stale', stale); }

    // odometer
    let odoVal = trip * u.dk;
    if (resetAnim) {
      const p = (t - resetAnim.t0) / 0.9;
      if (p >= 1) resetAnim = null;
      else odoVal = resetAnim.from * (1 - smooth(p)) + odoVal;
    }
    renderOdo(odoVal);

    // status
    const [state, text] = statusFor(t, gpsFresh, motionFresh);
    const key = state + text;
    if (key !== lastStatus) { lastStatus = key; statusEl.dataset.state = state; statusText.textContent = text; }

    requestAnimationFrame(frame);
  }

  // ---------- settings UI ----------
  const dash = $('#dash'), dialog = $('#settings');
  const SOURCE_HINT = {
    auto: 'Accelerometer for instant response, kept honest by GPS. Falls back to whichever one is working.',
    accel: 'Dead reckoning only. Responsive, but it drifts within minutes. Location is never requested.',
    gps: 'Most accurate over time. Updates about once a second and lags slightly.',
  };

  function apply() {
    const body = document.body;
    body.dataset.theme = S.theme;
    body.dataset.view = S.view;
    if (S.accent) body.style.setProperty('--accent', S.accent);
    else body.style.removeProperty('--accent');
    dash.classList.toggle('mirror', !!S.mirror);
    const u = UNITS[S.units];
    $('#unit').textContent = u.label;
    $('#tripUnit').textContent = u.dist;
    buildGauge(); buildOdo(); buildLED();
    lastTxt = null; lastFrac = -1;
    const meta = document.querySelector('meta[name="theme-color"]');
    if (meta) meta.content = getComputedStyle(body).getPropertyValue('--bg').trim();
    sensors(); wake();
    syncControls();
  }

  function syncControls() {
    document.querySelectorAll('[data-key]').forEach((group) => {
      const key = group.dataset.key;
      if (group.type === 'checkbox') { group.checked = !!S[key]; return; }
      group.querySelectorAll('button[data-v]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.v === String(S[key]))));
    });
    $('#gaugeMax').value = String(S.gaugeMax);
    $('#accent').value = S.accent || getComputedStyle(document.body).getPropertyValue('--accent').trim() || '#5ef2cf';
    $('#sourceHint').textContent = SOURCE_HINT[S.source];
    $('#demo').checked = demo;
  }

  function set(key, value) {
    if (key === 'source' && value !== 'gps') askMotion();   // still inside the tap, so iOS will prompt
    S[key] = value; saveSettings(); apply();
  }

  document.querySelectorAll('.seg[data-key], .swatches[data-key]').forEach((group) => {
    group.addEventListener('click', (e) => {
      const b = e.target.closest('button[data-v]');
      if (!b) return;
      set(group.dataset.key, 'num' in group.dataset ? Number(b.dataset.v) : b.dataset.v);
    });
  });
  document.querySelectorAll('input[type="checkbox"][data-key]').forEach((cb) => {
    cb.addEventListener('change', () => set(cb.dataset.key, cb.checked));
  });
  $('#gaugeMax').addEventListener('change', (e) => set('gaugeMax', e.target.value));
  $('#accent').addEventListener('input', (e) => { S.accent = e.target.value; saveSettings(); document.body.style.setProperty('--accent', S.accent); });
  $('#accent').addEventListener('change', () => apply());
  $('#clearAccent').addEventListener('click', () => set('accent', ''));
  $('#resetTrip').addEventListener('click', resetTrip);
  $('#demo').addEventListener('change', (e) => {
    demo = e.target.checked; demoT0 = now();
    if (!demo) { E.s = 0; E.P00 = 1; }
  });
  $('#openSettings').addEventListener('click', () => { syncControls(); renderDiag(); dialog.showModal(); });
  dialog.addEventListener('click', (e) => { if (e.target === dialog) dialog.close(); });   // tap the backdrop

  // Tap the speed to cycle displays
  const VIEWS = ['number', 'gauge', 'led'];
  $('#speedo').addEventListener('click', () => set('view', VIEWS[(VIEWS.indexOf(S.view) + 1) % VIEWS.length]));

  // Hold the odometer to reset
  const tripEl = $('#trip');
  let holdTimer = null;
  const cancelHold = () => { clearTimeout(holdTimer); holdTimer = null; tripEl.classList.remove('holding'); };
  tripEl.addEventListener('pointerdown', () => {
    cancelHold();
    tripEl.classList.add('holding');
    holdTimer = setTimeout(() => { cancelHold(); resetTrip(); }, 1000);
  });
  ['pointerup', 'pointerleave', 'pointercancel'].forEach((ev) => tripEl.addEventListener(ev, cancelHold));
  tripEl.addEventListener('contextmenu', (e) => e.preventDefault());

  // Live sensor details inside the settings sheet
  function renderDiag() {
    const t = now();
    const rows = [
      ['Motion', E.motionOK && t - E.lastMotionT < 1 ? `${Math.round(E.hz)} Hz` : (S.source === 'gps' ? 'Off' : 'No data')],
      ['GPS', S.source === 'accel' ? 'Off' : E.gpsErr === 1 ? 'Permission denied'
        : E.gpsAcc == null ? 'No fix yet'
        : `±${Math.round(E.gpsAcc)} m${E.gpsDerived ? ', speed from positions' : ''}`],
      ['Forward axis', E.fwdFrom === 'gps' ? `Learned from GPS (${E.fwdConf})` : E.fwdFrom === 'launch' ? 'Learned from pull-away' : 'Guessed from tilt'],
      ['Accel bias', `${E.b.toFixed(3)} m/s²`],
    ];
    $('#diag').replaceChildren(...rows.flatMap(([k, v]) => {
      const dt = document.createElement('dt'); dt.textContent = k;
      const dd = document.createElement('dd'); dd.textContent = v;
      return [dt, dd];
    }));
  }
  setInterval(() => { if (dialog.open) renderDiag(); }, 500);

  // ---------- boot ----------
  apply();
  requestAnimationFrame(frame);
  boot();
  if (new URLSearchParams(location.search).has('debug')) window.speedo = { E, S };   // poke at it from the console
  // iOS will only grant a wake lock after a tap
  document.addEventListener('pointerdown', () => wake(), { once: true });

  if ('serviceWorker' in navigator && window.isSecureContext) {
    window.addEventListener('load', () => navigator.serviceWorker.register('./sw.js').catch(() => {}));
  }
})();
