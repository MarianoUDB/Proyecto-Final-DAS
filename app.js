/**
 * AERO-GNC | Controlador de Interfaz Interactiva Web y Renderizadores de Instrumentación EFIS
 */

// Estado global de la aplicación
const state = {
    simData: null,
    currentIndex: 0,
    isPlaying: true,
    playbackSpeed: 1.0,
    animationFrameId: null,
    lastFrameTime: performance.now(),
    aircraftId: "b737",
};

// Elementos del DOM
const els = {
    pfdCanvas: document.getElementById("pfd-canvas"),
    runwayCanvas: document.getElementById("runway-canvas"),
    chartGlideslope: document.getElementById("chart-glideslope"),
    chartDots: document.getElementById("chart-dots"),
    chartControls: document.getElementById("chart-controls"),
    
    // Sliders y valores
    slHeadwind: document.getElementById("sl-headwind"),
    valHeadwind: document.getElementById("val-headwind"),
    slCrosswind: document.getElementById("sl-crosswind"),
    valCrosswind: document.getElementById("val-crosswind"),
    chkMicroburst: document.getElementById("chk-microburst"),
    slMicroU: document.getElementById("sl-micro-u"),
    valMicroU: document.getElementById("val-micro-u"),
    slMicroW: document.getElementById("sl-micro-w"),
    valMicroW: document.getElementById("val-micro-w"),
    slTurbulence: document.getElementById("sl-turbulence"),
    valTurbulence: document.getElementById("val-turbulence"),

    // Botones
    btnRunSim: document.getElementById("btn-run-sim"),
    btnPlayPause: document.getElementById("btn-play-pause"),
    btnSpeed: document.getElementById("btn-speed"),
    btnRestart: document.getElementById("btn-restart"),
    timelineSlider: document.getElementById("timeline-slider"),
    aircraftSelector: document.getElementById("aircraft-selector"),
    btnValidationModal: document.getElementById("btn-validation-modal"),
    btnCloseModal: document.getElementById("btn-close-modal"),
    btnModalOk: document.getElementById("btn-modal-ok"),
    validationModal: document.getElementById("validation-modal"),

    // Lecturas de telemetría
    simTimeDisplay: document.getElementById("sim-time-display"),
    flightPhaseBadge: document.getElementById("flight-phase-badge"),
    readoutTas: document.getElementById("readout-tas"),
    readoutAlt: document.getElementById("readout-alt"),
    readoutVs: document.getElementById("readout-vs"),
    readoutAlphaBeta: document.getElementById("readout-alpha-beta"),
    readoutNz: document.getElementById("readout-nz"),
    readoutThrottle: document.getElementById("readout-throttle"),

    // Compuertas FAA
    gateImcStatus: document.getElementById("gate-imc-status"),
    gateImcDetails: document.getElementById("gate-imc-details"),
    gateVmcStatus: document.getElementById("gate-vmc-status"),
    gateVmcDetails: document.getElementById("gate-vmc-details"),
    touchdownStatus: document.getElementById("touchdown-status"),
    touchdownDetails: document.getElementById("touchdown-details"),
    eventsList: document.getElementById("events-list"),

    // Modal
    tbodyBenchmark: document.getElementById("tbody-benchmark"),
    tbodyApprox: document.getElementById("tbody-approx"),
    rk4VerificationBox: document.getElementById("rk4-verification-box"),
};

// Inicialización de la aplicación
window.addEventListener("DOMContentLoaded", () => {
    setupEventListeners();
    runSimulation();
});

function setupEventListeners() {
    // Sincronización de sliders con texto
    els.slHeadwind.addEventListener("input", (e) => {
        const v = parseFloat(e.target.value);
        els.valHeadwind.textContent = `${v.toFixed(1)} m/s (~${(v * 1.94).toFixed(0)} kts)`;
    });
    els.slCrosswind.addEventListener("input", (e) => {
        const v = parseFloat(e.target.value);
        els.valCrosswind.textContent = `${v.toFixed(1)} m/s (~${(v * 1.94).toFixed(0)} kts)`;
    });
    els.slMicroU.addEventListener("input", (e) => {
        const v = parseFloat(e.target.value);
        els.valMicroU.textContent = `${v.toFixed(1)} m/s (~${(v * 1.94).toFixed(0)} kts)`;
    });
    els.slMicroW.addEventListener("input", (e) => {
        const v = parseFloat(e.target.value);
        els.valMicroW.textContent = `${v.toFixed(1)} m/s (~${(v * 196.85).toFixed(0)} fpm)`;
    });
    els.slTurbulence.addEventListener("input", (e) => {
        const v = parseFloat(e.target.value);
        els.valTurbulence.textContent = `${v.toFixed(1)} m/s (${v < 0.5 ? "Ligera" : v < 1.0 ? "Moderada" : "Severa"})`;
    });

    els.btnRunSim.addEventListener("click", runSimulation);
    els.aircraftSelector.addEventListener("change", (e) => {
        state.aircraftId = e.target.value;
        runSimulation();
    });

    // Playback
    els.btnPlayPause.addEventListener("click", () => {
        state.isPlaying = !state.isPlaying;
        els.btnPlayPause.textContent = state.isPlaying ? "PAUSA" : "PLAY";
    });

    els.btnSpeed.addEventListener("click", () => {
        if (state.playbackSpeed === 1.0) state.playbackSpeed = 2.0;
        else if (state.playbackSpeed === 2.0) state.playbackSpeed = 5.0;
        else state.playbackSpeed = 1.0;
        els.btnSpeed.textContent = `${state.playbackSpeed}x`;
    });

    els.btnRestart.addEventListener("click", () => {
        state.currentIndex = 0;
        state.isPlaying = true;
        els.btnPlayPause.textContent = "PAUSA";
    });

    els.timelineSlider.addEventListener("input", (e) => {
        if (!state.simData) return;
        const total = state.simData.data.time.length;
        state.currentIndex = Math.min(total - 1, Math.floor((parseFloat(e.target.value) / 100.0) * total));
    });

    // Modal
    els.btnValidationModal.addEventListener("click", openValidationModal);
    els.btnCloseModal.addEventListener("click", () => els.validationModal.classList.add("hidden"));
    els.btnModalOk.addEventListener("click", () => els.validationModal.classList.add("hidden"));
}

async function runSimulation() {
    els.btnRunSim.textContent = "SIMULANDO...";
    els.btnRunSim.disabled = true;

    const payload = {
        aircraft_id: state.aircraftId,
        headwind: parseFloat(els.slHeadwind.value),
        crosswind: parseFloat(els.slCrosswind.value),
        microburst: els.chkMicroburst.checked,
        microburst_delta_u: parseFloat(els.slMicroU.value),
        microburst_w_down: parseFloat(els.slMicroW.value),
        turbulence: parseFloat(els.slTurbulence.value),
        lateral_offset: 18.0
    };

    let data = null;

    // 1. Intentar llamar al backend local de Python (si está corriendo)
    try {
        const res = await fetch("/api/simulate", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload)
        });
        if (res.ok) {
            data = await res.json();
        }
    } catch (err) {
        // En GitHub Pages no hay servidor Python, se recurre a datos estáticos
    }

    // 2. Si no hay backend, cargar escenario estático precalculado correspondiente
    if (!data || data.status !== "success") {
        try {
            let scId = `${state.aircraftId}_nominal`;
            if (payload.microburst) {
                scId = `${state.aircraftId}_microburst`;
            } else if (payload.crosswind > 4.0) {
                scId = `${state.aircraftId}_crosswind`;
            }

            console.log(`[AERO-GNC] Modo estático activo. Cargando escenario: ${scId}`);

            const candidates = [
                `data/sim_${scId}.json`,
                `./data/sim_${scId}.json`,
                `data/sim_default.json`,
                `./data/sim_default.json`
            ];

            for (const url of candidates) {
                try {
                    const staticRes = await fetch(url);
                    if (staticRes.ok) {
                        data = await staticRes.json();
                        console.log(`[AERO-GNC] Datos cargados exitosamente desde: ${url}`);
                        break;
                    }
                } catch (e) {
                    // Siguiente candidato
                }
            }
        } catch (staticErr) {
            console.error("Error al cargar escenario estático:", staticErr);
        }
    }

    if (data && data.status === "success") {
        state.simData = data;
        state.currentIndex = 0;
        state.isPlaying = true;
        els.btnPlayPause.textContent = "PAUSA";
        updateStaticSummaries(data);
        startAnimationLoop();
    } else {
        alert("No se pudo cargar la simulación. Verifique la conexión.");
    }

    els.btnRunSim.textContent = "SIMULAR VUELO";
    els.btnRunSim.disabled = false;
}

function updateStaticSummaries(data) {
    // Compuerta IMC
    if (data.imc_gate) {
        const g = data.imc_gate;
        els.gateImcStatus.textContent = g.is_stabilized ? "ESTABILIZADA" : "DESESTABILIZADA";
        els.gateImcStatus.className = `badge ${g.is_stabilized ? "phase-normal" : "phase-critical"}`;
        els.gateImcDetails.innerHTML = `GS: ${g.gs_dots > 0 ? "+" : ""}${g.gs_dots} dots | LOC: ${g.loc_dots > 0 ? "+" : ""}${g.loc_dots} dots<br>ΔV: ${g.speed_error_kts > 0 ? "+" : ""}${g.speed_error_kts} kts | Vz: ${g.descent_rate_fpm} fpm`;
    }

    // Compuerta VMC
    if (data.vmc_gate) {
        const g = data.vmc_gate;
        els.gateVmcStatus.textContent = g.is_stabilized ? "ESTABILIZADA" : "DESESTABILIZADA";
        els.gateVmcStatus.className = `badge ${g.is_stabilized ? "phase-normal" : "phase-critical"}`;
        els.gateVmcDetails.innerHTML = `GS: ${g.gs_dots > 0 ? "+" : ""}${g.gs_dots} dots | LOC: ${g.loc_dots > 0 ? "+" : ""}${g.loc_dots} dots<br>ΔV: ${g.speed_error_kts > 0 ? "+" : ""}${g.speed_error_kts} kts | Vz: ${g.descent_rate_fpm} fpm`;
    }

    // Touchdown
    if (data.touchdown && data.touchdown_info) {
        const td = data.touchdown_info;
        els.touchdownStatus.textContent = "TOUCHDOWN EXITOSO";
        els.touchdownStatus.className = "badge phase-normal";
        els.touchdownDetails.innerHTML = `Distancia: ${td.x_touchdown_m.toFixed(1)} m | Desviación eje: ${td.y_touchdown_m.toFixed(1)} m<br>Tasa de toque: ${td.vertical_speed_fpm.toFixed(0)} ft/min | Cabeceo: ${td.pitch_deg.toFixed(1)}°`;
    }

    // Registro de eventos
    els.eventsList.innerHTML = "";
    data.events.forEach((ev) => {
        const li = document.createElement("li");
        li.textContent = ev;
        els.eventsList.appendChild(li);
    });
}

function startAnimationLoop() {
    if (state.animationFrameId) cancelAnimationFrame(state.animationFrameId);
    state.lastFrameTime = performance.now();

    function loop(now) {
        const dtReal = (now - state.lastFrameTime) / 1000.0;
        state.lastFrameTime = now;

        if (state.isPlaying && state.simData) {
            const timeStepSim = 0.05; // Paso de telemetría muestreado
            const advanceSteps = (dtReal * state.playbackSpeed) / timeStepSim;
            state.currentIndex = Math.min(
                state.simData.data.time.length - 1,
                state.currentIndex + advanceSteps
            );
            if (state.currentIndex >= state.simData.data.time.length - 1) {
                state.isPlaying = false;
                els.btnPlayPause.textContent = "PLAY";
            }
        }

        renderCurrentFrame();
        state.animationFrameId = requestAnimationFrame(loop);
    }

    state.animationFrameId = requestAnimationFrame(loop);
}

function renderCurrentFrame() {
    if (!state.simData) return;
    const idx = Math.floor(state.currentIndex);
    const d = state.simData.data;
    const total = d.time.length;

    // Actualizar slider
    els.timelineSlider.value = ((idx / (total - 1)) * 100).toFixed(1);

    const t = d.time[idx];
    const s = d.state[idx];
    const c = d.controls[idx];
    const telem = d.telemetry;

    const u = s[0], v = s[1], w = s[2];
    const p = s[3], q = s[4], r = s[5];
    const phi = s[6], theta = s[7], psi = s[8];
    const x_n = s[9], y_e = s[10], z_d = s[11];
    const alt_m = telem.altitude_m[idx];
    const alt_ft = alt_m * 3.28084;
    const tas_kts = telem.v_tas[idx] * 1.94384;
    const vs_fpm = telem.vertical_speed_fpm[idx];
    const gs_dots = telem.gs_dots[idx];
    const loc_dots = telem.loc_dots[idx];

    // Actualizar lecturas
    els.simTimeDisplay.textContent = `T+ ${t.toFixed(2)} s`;
    els.readoutTas.textContent = tas_kts.toFixed(1);
    els.readoutAlt.textContent = Math.round(alt_ft);
    els.readoutVs.textContent = Math.round(vs_fpm);
    els.readoutAlphaBeta.textContent = `${telem.alpha_deg[idx].toFixed(1)}° / ${telem.beta_deg[idx].toFixed(1)}°`;
    els.readoutNz.textContent = telem.n_z[idx].toFixed(2);
    els.readoutThrottle.textContent = (c[3] * 100).toFixed(1);

    // Estado operacional
    if (alt_m <= 1.0) {
        els.flightPhaseBadge.textContent = "TOUCHDOWN // EN PISTA";
        els.flightPhaseBadge.className = "badge phase-normal";
    } else if (alt_m < 15.0) {
        els.flightPhaseBadge.textContent = "MODO RECOGIDA (FLARE)";
        els.flightPhaseBadge.className = "badge phase-normal";
    } else if (Math.abs(gs_dots) > 1.0 || Math.abs(loc_dots) > 1.0) {
        els.flightPhaseBadge.textContent = "CORRECCIÓN POR CIZALLADURA";
        els.flightPhaseBadge.className = "badge phase-warning";
    } else {
        els.flightPhaseBadge.textContent = "APROXIMACIÓN ESTABILIZADA";
        els.flightPhaseBadge.className = "badge phase-normal";
    }

    // Renderizar PFD, Perspectiva 3D y Gráficos
    renderPFD(phi, theta, tas_kts, alt_ft, vs_fpm, gs_dots, loc_dots, telem.theta_cmd_deg[idx], telem.phi_cmd_deg[idx]);
    renderRunwayPerspective(x_n, y_e, alt_m, phi, theta, psi);
    renderStripCharts(idx);
}

/* =====================================================================
   RENDERIZADOR EFIS PRIMARY FLIGHT DISPLAY (PFD)
   ===================================================================== */
function renderPFD(phi, theta, tas_kts, alt_ft, vs_fpm, gs_dots, loc_dots, theta_cmd_deg, phi_cmd_deg) {
    const cv = els.pfdCanvas;
    const ctx = cv.getContext("2d");
    const w = cv.width, h = cv.height;
    const cx = w / 2, cy = h / 2;

    ctx.clearRect(0, 0, w, h);

    // 1. Horizonte Artificial (Cielo / Tierra con Alabeo y Cabeceo)
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-phi);

    const pixelsPerDegPitch = 4.0;
    const pitchOffset = theta * (180.0 / Math.PI) * pixelsPerDegPitch;

    // Cielo
    ctx.fillStyle = "#0369a1";
    ctx.fillRect(-w, -h * 2 + pitchOffset, w * 2, h * 2);

    // Tierra
    ctx.fillStyle = "#78350f";
    ctx.fillRect(-w, pitchOffset, w * 2, h * 2);

    // Línea de Horizonte
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(-w, pitchOffset);
    ctx.lineTo(w, pitchOffset);
    ctx.stroke();

    // Escala de Cabeceo (Pitch Ladder)
    ctx.strokeStyle = "#ffffff";
    ctx.fillStyle = "#ffffff";
    ctx.lineWidth = 1.5;
    ctx.font = "10px 'Share Tech Mono', monospace";
    ctx.textAlign = "center";

    for (let deg = -20; deg <= 25; deg += 5) {
        if (deg === 0) continue;
        const y = pitchOffset - deg * pixelsPerDegPitch;
        const lineLen = deg % 10 === 0 ? 36 : 20;
        ctx.beginPath();
        ctx.moveTo(-lineLen, y);
        ctx.lineTo(lineLen, y);
        ctx.stroke();
        if (deg % 10 === 0) {
            ctx.fillText(`${Math.abs(deg)}`, -lineLen - 12, y + 3);
            ctx.fillText(`${Math.abs(deg)}`, lineLen + 12, y + 3);
        }
    }

    // Director de Vuelo (Flight Director - Barras magenta)
    if (theta_cmd_deg !== undefined) {
        const fdPitchY = pitchOffset - theta_cmd_deg * pixelsPerDegPitch;
        ctx.strokeStyle = "#c084fc";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(-40, fdPitchY);
        ctx.lineTo(40, fdPitchY);
        ctx.stroke();
    }

    ctx.restore();

    // 2. Retícula fija de la aeronave (Aircraft Reference Symbol / W)
    ctx.strokeStyle = "#facc15";
    ctx.fillStyle = "#facc15";
    ctx.lineWidth = 3.5;
    ctx.beginPath();
    // Ala izquierda
    ctx.moveTo(cx - 50, cy);
    ctx.lineTo(cx - 20, cy);
    ctx.lineTo(cx - 20, cy + 8);
    // Centro
    ctx.moveTo(cx - 5, cy);
    ctx.lineTo(cx + 5, cy);
    // Ala derecha
    ctx.moveTo(cx + 20, cy + 8);
    ctx.lineTo(cx + 20, cy);
    ctx.lineTo(cx + 50, cy);
    ctx.stroke();
    ctx.fillRect(cx - 3, cy - 3, 6, 6);

    // 3. Cinta de Velocidad (Airspeed Tape - Izquierda)
    ctx.fillStyle = "rgba(15, 23, 42, 0.85)";
    ctx.fillRect(0, 0, 75, h);
    ctx.strokeStyle = "rgba(255, 255, 255, 0.2)";
    ctx.strokeRect(0, 0, 75, h);

    const pxPerKnot = 2.2;
    ctx.strokeStyle = "#ffffff";
    ctx.fillStyle = "#ffffff";
    ctx.font = "11px 'Share Tech Mono', monospace";
    ctx.textAlign = "right";

    const vMin = Math.floor((tas_kts - 35) / 10) * 10;
    const vMax = vMin + 70;
    for (let spd = vMin; spd <= vMax; spd += 10) {
        const y = cy - (spd - tas_kts) * pxPerKnot;
        if (y >= 10 && y <= h - 10) {
            ctx.beginPath();
            ctx.moveTo(65, y);
            ctx.lineTo(75, y);
            ctx.stroke();
            ctx.fillText(`${spd}`, 60, y + 4);
        }
    }

    // Ventana digital de velocidad actual
    ctx.fillStyle = "#000000";
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 1.8;
    ctx.fillRect(5, cy - 14, 66, 28);
    ctx.strokeRect(5, cy - 14, 66, 28);
    ctx.fillStyle = "#38bdf8";
    ctx.font = "bold 16px 'Share Tech Mono', monospace";
    ctx.textAlign = "center";
    ctx.fillText(`${tas_kts.toFixed(0)}`, 38, cy + 6);

    // 4. Cinta de Altitud (Altitude Tape - Derecha)
    const altTapeX = w - 85;
    ctx.fillStyle = "rgba(15, 23, 42, 0.85)";
    ctx.fillRect(altTapeX, 0, 85, h);
    ctx.strokeStyle = "rgba(255, 255, 255, 0.2)";
    ctx.strokeRect(altTapeX, 0, 85, h);

    const pxPer100Ft = 18.0;
    const altMin = Math.floor((alt_ft - 400) / 100) * 100;
    const altMax = altMin + 800;
    ctx.textAlign = "left";

    for (let altVal = altMin; altVal <= altMax; altVal += 100) {
        const y = cy - ((altVal - alt_ft) / 100.0) * pxPer100Ft;
        if (y >= 10 && y <= h - 10) {
            ctx.beginPath();
            ctx.moveTo(altTapeX, y);
            ctx.lineTo(altTapeX + 10, y);
            ctx.stroke();
            ctx.fillText(`${altVal}`, altTapeX + 15, y + 4);
        }
    }

    // Ventana digital de altitud actual
    ctx.fillStyle = "#000000";
    ctx.strokeStyle = "#22c55e";
    ctx.lineWidth = 1.8;
    ctx.fillRect(altTapeX + 8, cy - 14, 72, 28);
    ctx.strokeRect(altTapeX + 8, cy - 14, 72, 28);
    ctx.fillStyle = "#22c55e";
    ctx.font = "bold 15px 'Share Tech Mono', monospace";
    ctx.textAlign = "center";
    ctx.fillText(`${Math.round(alt_ft)}`, altTapeX + 44, cy + 6);

    // 5. Agujas ILS (Glideslope Vertical & Localizer Horizontal)
    // Glideslope (Escala de rombo en borde derecho del horizonte)
    const gsScaleX = altTapeX - 16;
    ctx.fillStyle = "#ffffff";
    for (let dot = -2; dot <= 2; dot++) {
        const dy = cy + dot * 25;
        ctx.beginPath();
        ctx.arc(gsScaleX, dy, dot === 0 ? 3 : 2, 0, Math.PI * 2);
        ctx.fill();
    }
    // Rombo Glideslope (Magenta)
    const gsDiamondY = cy - gs_dots * 25;
    ctx.fillStyle = "#c084fc";
    ctx.beginPath();
    ctx.moveTo(gsScaleX, gsDiamondY - 6);
    ctx.lineTo(gsScaleX + 6, gsDiamondY);
    ctx.lineTo(gsScaleX, gsDiamondY + 6);
    ctx.lineTo(gsScaleX - 6, gsDiamondY);
    ctx.closePath();
    ctx.fill();

    // Localizer (Escala horizontal en parte inferior del horizonte)
    const locScaleY = h - 35;
    for (let dot = -2; dot <= 2; dot++) {
        const dx = cx + dot * 25;
        ctx.beginPath();
        ctx.arc(dx, locScaleY, dot === 0 ? 3 : 2, 0, Math.PI * 2);
        ctx.fill();
    }
    // Rombo Localizer (Magenta)
    const locDiamondX = cx + loc_dots * 25;
    ctx.fillStyle = "#c084fc";
    ctx.beginPath();
    ctx.moveTo(locDiamondX - 6, locScaleY);
    ctx.lineTo(locDiamondX, locScaleY - 6);
    ctx.lineTo(locDiamondX + 6, locScaleY);
    ctx.lineTo(locDiamondX, locScaleY + 6);
    ctx.closePath();
    ctx.fill();
}

/* =====================================================================
   RENDERIZADOR 3D DE APROXIMACIÓN A PISTA
   ===================================================================== */
function renderRunwayPerspective(x_m, y_m, alt_m, phi, theta, psi) {
    const cv = els.runwayCanvas;
    const ctx = cv.getContext("2d");
    const w = cv.width, h = cv.height;

    ctx.fillStyle = "#050914";
    ctx.fillRect(0, 0, w, h);

    // Suelo nocturno con horizonte
    const horizonY = h * 0.45;
    ctx.fillStyle = "#0c1729";
    ctx.fillRect(0, horizonY, w, h - horizonY);

    // Función de proyección en perspectiva simple
    // Origen de pista: [x=0, y=0, z=0]. Avión en [x_m, y_m, alt_m].
    function project(px, py, pz) {
        const dx = px - x_m; // Avance longitudinal
        if (dx <= 1.0) return null; // Detrás de la cámara
        const dy = py - y_m;
        const dz = pz - alt_m;

        const fov = 350.0;
        const sx = w / 2 + (dy / dx) * fov;
        const sy = horizonY - (dz / dx) * fov;
        return { x: sx, y: sy, depth: dx };
    }

    // Pista de aterrizaje (Longitud 3500m, Ancho 45m)
    const rwyLen = 3500.0, rwyW = 45.0;
    const p1 = project(0, -rwyW / 2, 0);
    const p2 = project(0, rwyW / 2, 0);
    const p3 = project(rwyLen, rwyW / 2, 0);
    const p4 = project(rwyLen, -rwyW / 2, 0);

    if (p1 && p2 && p3 && p4) {
        // Superficie de asfalto
        ctx.fillStyle = "#1e293b";
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.lineTo(p3.x, p3.y);
        ctx.lineTo(p4.x, p4.y);
        ctx.closePath();
        ctx.fill();

        // Luces de borde de pista (blancas)
        ctx.strokeStyle = "#fef08a";
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y); ctx.lineTo(p4.x, p4.y);
        ctx.moveTo(p2.x, p2.y); ctx.lineTo(p3.x, p3.y);
        ctx.stroke();

        // Umbral de pista (Luces verdes)
        ctx.strokeStyle = "#22c55e";
        ctx.lineWidth = 3.0;
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y);
        ctx.stroke();

        // Línea central de pista (discontinua)
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 1.5;
        for (let x_seg = 100; x_seg < 3000; x_seg += 120) {
            const pa = project(x_seg, 0, 0);
            const pb = project(x_seg + 60, 0, 0);
            if (pa && pb) {
                ctx.beginPath();
                ctx.moveTo(pa.x, pa.y);
                ctx.lineTo(pb.x, pb.y);
                ctx.stroke();
            }
        }

        // Zona de contacto (Touchdown Zone Aiming Point a 300m)
        const tdzA = project(300, -rwyW * 0.3, 0);
        const tdzB = project(300, rwyW * 0.3, 0);
        if (tdzA && tdzB) {
            ctx.fillStyle = "#ffffff";
            ctx.fillRect(tdzA.x - 4, tdzA.y - 2, 8, 4);
            ctx.fillRect(tdzB.x - 4, tdzB.y - 2, 8, 4);
        }
    }

    // Luces de aproximación PAPI / ALS
    const alsDist = -300.0;
    const alsP = project(alsDist, 0, 0);
    if (alsP && alsP.y > horizonY) {
        ctx.fillStyle = "#ef4444";
        ctx.beginPath();
        ctx.arc(alsP.x, alsP.y, 3, 0, Math.PI * 2);
        ctx.fill();
    }
}

/* =====================================================================
   RENDERIZADOR DE STRIP CHARTS DINÁMICAS
   ===================================================================== */
function renderStripCharts(currentIdx) {
    const d = state.simData.data;
    const len = d.time.length;

    // 1. Chart Glideslope
    renderSingleChart(
        els.chartGlideslope,
        d.state.map((s) => s[9]), // X dist
        [
            { data: d.telemetry.altitude_m, color: "#38bdf8", width: 2.0 },
            { data: d.telemetry.h_nominal, color: "#ffffff", dash: [4, 4], width: 1.5 }
        ],
        currentIdx,
        "Altitud [m]"
    );

    // 2. Chart ILS Dots
    renderSingleChart(
        els.chartDots,
        d.time,
        [
            { data: d.telemetry.gs_dots, color: "#ef4444", width: 1.8 },
            { data: d.telemetry.loc_dots, color: "#c084fc", width: 1.8 }
        ],
        currentIdx,
        "Dots",
        [-2.5, 2.5]
    );

    // 3. Chart Mandos y Acelerador
    renderSingleChart(
        els.chartControls,
        d.time,
        [
            { data: d.controls.map((c) => c[0] * (180 / Math.PI)), color: "#38bdf8", width: 1.2 }, // de
            { data: d.controls.map((c) => c[1] * (180 / Math.PI)), color: "#22c55e", width: 1.2 }, // da
            { data: d.controls.map((c) => c[2] * (180 / Math.PI)), color: "#f97316", width: 1.2 }, // dr
            { data: d.controls.map((c) => c[3] * 100), color: "#ef4444", dash: [3, 3], width: 1.2 }  // th
        ],
        currentIdx,
        "Mando"
    );
}

function renderSingleChart(canvas, xData, seriesList, currentIdx, yLabel, fixedYRange = null) {
    const ctx = canvas.getContext("2d");
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    const padLeft = 40, padRight = 10, padTop = 10, padBottom = 20;
    const plotW = w - padLeft - padRight;
    const plotH = h - padTop - padBottom;

    // Rango Y
    let yMin = fixedYRange ? fixedYRange[0] : Infinity;
    let yMax = fixedYRange ? fixedYRange[1] : -Infinity;

    if (!fixedYRange) {
        seriesList.forEach((s) => {
            s.data.forEach((val) => {
                if (val < yMin) yMin = val;
                if (val > yMax) yMax = val;
            });
        });
        const yMargin = Math.max(0.1, (yMax - yMin) * 0.1);
        yMin -= yMargin;
        yMax += yMargin;
    }

    const n = xData.length;
    function getX(i) { return padLeft + (i / (n - 1)) * plotW; }
    function getY(val) { return padTop + plotH - ((val - yMin) / (yMax - yMin)) * plotH; }

    // Fondo y cuadrícula
    ctx.strokeStyle = "rgba(255, 255, 255, 0.08)";
    ctx.lineWidth = 1;
    ctx.strokeRect(padLeft, padTop, plotW, plotH);

    // Línea cero si está en rango
    if (yMin <= 0 && yMax >= 0) {
        ctx.strokeStyle = "rgba(255, 255, 255, 0.2)";
        const y0 = getY(0);
        ctx.beginPath();
        ctx.moveTo(padLeft, y0);
        ctx.lineTo(padLeft + plotW, y0);
        ctx.stroke();
    }

    // Dibujar curvas
    seriesList.forEach((s) => {
        ctx.save();
        ctx.strokeStyle = s.color;
        ctx.lineWidth = s.width;
        if (s.dash) ctx.setLineDash(s.dash);
        ctx.beginPath();
        for (let i = 0; i < n; i++) {
            const px = getX(i);
            const py = getY(s.data[i]);
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        }
        ctx.stroke();
        ctx.restore();
    });

    // Cursor temporal actual (línea vertical amarilla)
    const curX = getX(currentIdx);
    ctx.strokeStyle = "#facc15";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(curX, padTop);
    ctx.lineTo(curX, padTop + plotH);
    ctx.stroke();
}

/* =====================================================================
   MODAL DE VALIDACIÓN CUANTITATIVA
   ===================================================================== */
async function openValidationModal() {
    els.validationModal.classList.remove("hidden");
    els.tbodyBenchmark.innerHTML = `<tr><td colspan="7">Calculando autovalores analíticos...</td></tr>`;

    let data = null;
    try {
        const res = await fetch(`/api/validation?aircraft_id=${state.aircraftId}`);
        if (res.ok) {
            data = await res.json();
        }
    } catch (err) {
        // Fallback para GitHub Pages
    }

    if (!data || data.status !== "success") {
        try {
            const staticRes = await fetch(`./data/val_${state.aircraftId}.json`);
            if (staticRes.ok) {
                data = await staticRes.json();
            }
        } catch (sErr) {
            console.error("Error al cargar validación estática:", sErr);
        }
    }

    if (data && data.status === "success") {
            // 1. Tabla Benchmark
            els.tbodyBenchmark.innerHTML = "";
            data.benchmark_comparison.forEach((r) => {
                const tr = document.createElement("tr");
                tr.innerHTML = `
                    <td><strong>${r.mode_name}</strong></td>
                    <td>${r.computed_eigenvalue}</td>
                    <td>${r.reference_eigenvalue}</td>
                    <td>${r.computed_wn.toFixed(4)}</td>
                    <td>${r.reference_wn.toFixed(4)}</td>
                    <td style="color: ${r.rel_error_wn_percent < 5 ? '#22c55e' : '#f59e0b'};">
                        ${r.rel_error_wn_percent.toFixed(2)}%
                    </td>
                    <td style="color: ${r.rel_error_eigenvalue_percent < 5 ? '#22c55e' : '#f59e0b'};">
                        ${r.rel_error_eigenvalue_percent.toFixed(2)}%
                    </td>
                `;
                els.tbodyBenchmark.appendChild(tr);
            });

            // 2. Tabla Aproximaciones 2-DOF
            els.tbodyApprox.innerHTML = "";
            data.analytical_approximations.forEach((a) => {
                const tr = document.createElement("tr");
                tr.innerHTML = `
                    <td><strong>${a.mode_name}</strong></td>
                    <td>${a.exact_4dof_wn.toFixed(4)}</td>
                    <td>${a.approx_2dof_wn.toFixed(4)}</td>
                    <td>${a.exact_4dof_zeta.toFixed(4)}</td>
                    <td>${a.approx_2dof_zeta.toFixed(4)}</td>
                    <td style="color: ${a.rel_error_wn_percent < 15 ? '#22c55e' : '#f59e0b'};">
                        ${a.rel_error_wn_percent.toFixed(2)}%
                    </td>
                `;
                els.tbodyApprox.appendChild(tr);
            });

            // 3. Verificación RK4
            const rk = data.rk4_verification;
            els.rk4VerificationBox.innerHTML = `
                Orden de convergencia experimental estimado: <strong>p = ${rk.estimated_order}</strong> (Teórico: 4.00)<br>
                Convergencia de cuarto orden comprobada: <strong>${rk.is_fourth_order ? "SÍ (O(dt^4) Verificado)" : "NO"}</strong><br>
                Error global relativo al final de la integración: <strong>${rk.final_error_percent.toExponential(4)} %</strong>
            `;
        }
    } catch (err) {
        console.error("Error al obtener datos de validación:", err);
    }
}
