"""
Real-Time Monitoring Dashboard Web Application.
Provides live camera stream, 2D LiDAR radar, digital twin synchronization status,
and robot navigation telemetry over a lightweight HTTP server.
"""

import http.server
import socketserver
import json
import time
import cv2
import threading
import logging
from typing import Optional
from digital_twin.engine import DigitalTwinEngine

logger = logging.getLogger("Dashboard")

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Real-Time Digital Twin Laboratory Dashboard</title>
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-base: #0B0F17;
            --bg-card: rgba(18, 24, 38, 0.75);
            --bg-card-hover: rgba(28, 36, 56, 0.85);
            --border-glass: rgba(255, 255, 255, 0.08);
            --text-primary: #F0F4FC;
            --text-secondary: #94A3B8;
            --accent-cyan: #06B6D4;
            --accent-blue: #3B82F6;
            --accent-emerald: #10B981;
            --accent-amber: #F59E0B;
            --accent-rose: #F43F5E;
            --shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.37);
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            font-family: 'Outfit', sans-serif;
            background: radial-gradient(circle at 10% 20%, #0F172A 0%, #080B11 90%);
            color: var(--text-primary);
            min-height: 100vh;
            padding: 20px;
        }

        header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 24px;
            padding: 16px 24px;
            background: var(--bg-card);
            border: 1px solid var(--border-glass);
            border-radius: 16px;
            backdrop-filter: blur(16px);
            box-shadow: var(--shadow);
        }

        .header-title h1 {
            font-size: 1.5rem;
            font-weight: 700;
            background: linear-gradient(135deg, #38BDF8, #818CF8);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }

        .header-title p {
            font-size: 0.85rem;
            color: var(--text-secondary);
        }

        .status-badge {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 0.85rem;
            font-weight: 500;
            padding: 6px 14px;
            border-radius: 9999px;
            background: rgba(16, 185, 129, 0.15);
            border: 1px solid rgba(16, 185, 129, 0.3);
            color: var(--accent-emerald);
        }

        .status-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: var(--accent-emerald);
            box-shadow: 0 0 10px var(--accent-emerald);
            animation: pulse 2s infinite;
        }

        @keyframes pulse {
            0% { opacity: 0.5; transform: scale(0.9); }
            50% { opacity: 1; transform: scale(1.2); }
            100% { opacity: 0.5; transform: scale(0.9); }
        }

        .grid-container {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }

        .card {
            background: var(--bg-card);
            border: 1px solid var(--border-glass);
            border-radius: 16px;
            padding: 20px;
            backdrop-filter: blur(16px);
            box-shadow: var(--shadow);
            display: flex;
            flex-direction: column;
        }

        .card-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 16px;
        }

        .card-header h2 {
            font-size: 1.1rem;
            font-weight: 600;
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .view-frame {
            position: relative;
            width: 100%;
            height: 380px;
            background: #05080E;
            border-radius: 12px;
            overflow: hidden;
            display: flex;
            align-items: center;
            justify-content: center;
            border: 1px solid rgba(255, 255, 255, 0.05);
        }

        .view-frame img {
            width: 100%;
            height: 100%;
            object-fit: cover;
        }

        canvas {
            width: 100%;
            height: 100%;
        }

        .metrics-grid {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 12px;
            margin-top: 16px;
        }

        .metric-box {
            background: rgba(11, 15, 23, 0.6);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 10px;
            padding: 12px;
        }

        .metric-label {
            font-size: 0.75rem;
            color: var(--text-secondary);
            margin-bottom: 4px;
        }

        .metric-val {
            font-family: 'JetBrains Mono', monospace;
            font-size: 1.1rem;
            font-weight: 600;
            color: var(--text-primary);
        }

        .state-tag {
            font-family: 'JetBrains Mono', monospace;
            padding: 4px 10px;
            border-radius: 6px;
            font-size: 0.8rem;
            font-weight: 600;
        }

        .state-NAVIGATING { background: rgba(59, 130, 246, 0.2); color: #60A5FA; border: 1px solid #3B82F6; }
        .state-AVOIDING { background: rgba(245, 158, 11, 0.2); color: #FBBF24; border: 1px solid #F59E0B; }
        .state-BLOCKED { background: rgba(244, 63, 94, 0.2); color: #FB7185; border: 1px solid #F43F5E; }
        .state-GOAL_REACHED { background: rgba(16, 185, 129, 0.2); color: #34D399; border: 1px solid #10B981; }
        .state-IDLE { background: rgba(148, 163, 184, 0.2); color: #94A3B8; border: 1px solid #64748B; }

        .controls-row {
            display: flex;
            gap: 10px;
            margin-top: 14px;
        }

        button {
            background: linear-gradient(135deg, #2563EB, #1D4ED8);
            color: white;
            border: none;
            padding: 8px 14px;
            border-radius: 8px;
            font-size: 0.85rem;
            font-weight: 500;
            cursor: pointer;
            transition: all 0.2s ease;
        }

        button:hover {
            transform: translateY(-1px);
            filter: brightness(1.1);
        }

        button.danger {
            background: linear-gradient(135deg, #DC2626, #B91C1C);
        }

        table {
            width: 100%;
            border-collapse: collapse;
            font-size: 0.85rem;
            margin-top: 10px;
        }

        th, td {
            padding: 8px 12px;
            text-align: left;
            border-bottom: 1px solid rgba(255, 255, 255, 0.05);
        }

        th {
            color: var(--text-secondary);
            font-weight: 500;
        }
    </style>
</head>
<body>
    <header>
        <div class="header-title">
            <h1>Physical Room Digital Twin System</h1>
            <p>Real-Time LiDAR & Camera Perception with CoppeliaSim Autonomous Robot Navigation</p>
        </div>
        <div class="status-badge" id="system-badge">
            <span class="status-dot"></span>
            <span id="system-status-text">PIPELINE SYNCHRONISED</span>
        </div>
    </header>

    <div class="grid-container">
        <!-- Panel 1: Live Camera Feed -->
        <div class="card">
            <div class="card-header">
                <h2>📷 Physical Camera Feed</h2>
                <span id="cam-fps" style="font-size: 0.8rem; color: var(--text-secondary);">30.0 FPS</span>
            </div>
            <div class="view-frame">
                <img id="live-camera" src="/video_feed" alt="Camera Feed">
            </div>
            <div class="metrics-grid">
                <div class="metric-box">
                    <div class="metric-label">Resolution</div>
                    <div class="metric-val" id="cam-res">640x480</div>
                </div>
                <div class="metric-box">
                    <div class="metric-label">Hardware Device</div>
                    <div class="metric-val" id="cam-dev" style="font-size: 0.9rem;">/dev/video0</div>
                </div>
                <div class="metric-box">
                    <div class="metric-label">Status</div>
                    <div class="metric-val" id="cam-status" style="color: var(--accent-emerald);">ACTIVE</div>
                </div>
                <div class="metric-box">
                    <div class="metric-label">Last Timestamp</div>
                    <div class="metric-val" id="cam-ts" style="font-size: 0.8rem;">--</div>
                </div>
            </div>
        </div>

        <!-- Panel 2: 2D Spatial LiDAR & Digital Twin Map -->
        <div class="card">
            <div class="card-header">
                <h2>📡 2D Digital Twin & LiDAR Radar</h2>
                <span id="lidar-source" style="font-size: 0.8rem; color: var(--accent-cyan);">Hardware/Synthetic</span>
            </div>
            <div class="view-frame">
                <canvas id="radar-canvas" width="600" height="380"></canvas>
            </div>
            <div class="metrics-grid">
                <div class="metric-box">
                    <div class="metric-label">LiDAR Scan Points</div>
                    <div class="metric-val" id="lidar-pts">360</div>
                </div>
                <div class="metric-box">
                    <div class="metric-label">Sync Latency</div>
                    <div class="metric-val" id="sync-lat">-- ms</div>
                </div>
                <div class="metric-box">
                    <div class="metric-label">Active Obstacles</div>
                    <div class="metric-val" id="obs-count" style="color: var(--accent-amber);">0</div>
                </div>
                <div class="metric-box">
                    <div class="metric-label">CoppeliaSim RPC</div>
                    <div class="metric-val" id="sim-status" style="color: var(--accent-emerald);">CONNECTED</div>
                </div>
            </div>
        </div>
    </div>

    <!-- Panel 3: Autonomous Robot Telemetry & Interactive Obstacle Controls -->
    <div class="grid-container">
        <div class="card">
            <div class="card-header">
                <h2>🤖 Robot Navigation Telemetry</h2>
                <span class="state-tag state-IDLE" id="nav-state-badge">IDLE</span>
            </div>
            <div class="metrics-grid">
                <div class="metric-box">
                    <div class="metric-label">Robot Position (X, Y)</div>
                    <div class="metric-val" id="robot-pos">(-1.50, -1.00)</div>
                </div>
                <div class="metric-box">
                    <div class="metric-label">Heading (Yaw)</div>
                    <div class="metric-val" id="robot-yaw">0.0°</div>
                </div>
                <div class="metric-box">
                    <div class="metric-label">Wheel Speeds (L/R)</div>
                    <div class="metric-val" id="wheel-speeds">0.0 / 0.0</div>
                </div>
                <div class="metric-box">
                    <div class="metric-label">Mission Goal</div>
                    <div class="metric-val" id="robot-goal">(1.50, 1.00)</div>
                </div>
            </div>
            <div class="controls-row">
                <button onclick="injectObstacle(0.0, 0.0)">+ Place Obstacle in Robot Path (0, 0)</button>
                <button onclick="injectObstacle(0.6, 0.4)">+ Place Offset Obstacle (0.6, 0.4)</button>
                <button class="danger" onclick="clearObstacles()">Remove Physical Obstacles</button>
            </div>
        </div>

        <div class="card">
            <div class="card-header">
                <h2>📦 Tracked Obstacles in Virtual Twin</h2>
            </div>
            <table>
                <thead>
                    <tr>
                        <th>ID</th>
                        <th>Position [X, Y]</th>
                        <th>Radius</th>
                        <th>Confidence</th>
                        <th>Age</th>
                    </tr>
                </thead>
                <tbody id="obstacles-tbody">
                    <tr><td colspan="5" style="text-align: center; color: var(--text-secondary);">No obstacles detected</td></tr>
                </tbody>
            </table>
        </div>
    </div>

    <script>
        const canvas = document.getElementById("radar-canvas");
        const ctx = canvas.getContext("2d");

        function roomToCanvas(rx, ry) {
            // Room is 5.0m wide (-2.5 to 2.5), 4.0m long (-2.0 to 2.0)
            const scaleX = canvas.width / 5.6;
            const scaleY = canvas.height / 4.6;
            const scale = Math.min(scaleX, scaleY);
            const cx = canvas.width / 2.0 + (rx * scale);
            const cy = canvas.height / 2.0 - (ry * scale);
            return [cx, cy, scale];
        }

        function drawMap(data) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // 1. Draw Room Bounds
            const [w1, h1, s] = roomToCanvas(-2.5, 2.0);
            ctx.strokeStyle = "rgba(59, 130, 246, 0.4)";
            ctx.lineWidth = 2;
            ctx.strokeRect(w1, h1, 5.0 * s, 4.0 * s);

            // Coordinate grid axes
            const [cx, cy] = roomToCanvas(0, 0);
            ctx.strokeStyle = "rgba(255, 255, 255, 0.08)";
            ctx.beginPath();
            ctx.moveTo(cx, 0); ctx.lineTo(cx, canvas.height);
            ctx.moveTo(0, cy); ctx.lineTo(canvas.width, cy);
            ctx.stroke();

            // 2. Draw Goal
            if (data.robot && data.robot.goal) {
                const [gx, gy] = roomToCanvas(data.robot.goal[0], data.robot.goal[1]);
                ctx.fillStyle = "rgba(16, 185, 129, 0.3)";
                ctx.beginPath();
                ctx.arc(gx, gy, 0.20 * s, 0, Math.PI * 2);
                ctx.fill();
                ctx.strokeStyle = "#10B981";
                ctx.stroke();
                ctx.fillStyle = "#10B981";
                ctx.font = "10px Outfit";
                ctx.fillText("GOAL", gx - 12, gy - 12);
            }

            // 3. Draw Detected Obstacles
            if (data.digital_twin && data.digital_twin.obstacles) {
                data.digital_twin.obstacles.forEach(obs => {
                    const [ox, oy] = roomToCanvas(obs.centroid[0], obs.centroid[1]);
                    const radPx = Math.max(8, obs.radius * s);
                    
                    ctx.fillStyle = "rgba(244, 63, 94, 0.45)";
                    ctx.beginPath();
                    ctx.arc(ox, oy, radPx, 0, Math.PI * 2);
                    ctx.fill();
                    ctx.strokeStyle = "#F43F5E";
                    ctx.lineWidth = 2;
                    ctx.stroke();

                    ctx.fillStyle = "#FFFFFF";
                    ctx.font = "10px JetBrains Mono";
                    ctx.fillText(obs.id, ox - 14, oy - radPx - 4);
                });
            }

            // 4. Draw Robot
            if (data.robot && data.robot.pose) {
                const [rx, ry] = roomToCanvas(data.robot.pose[0], data.robot.pose[1]);
                const yaw = data.robot.pose[2];

                ctx.save();
                ctx.translate(rx, ry);
                ctx.rotate(-yaw);

                // Robot body
                ctx.fillStyle = "#38BDF8";
                ctx.beginPath();
                ctx.arc(0, 0, 10, 0, Math.PI * 2);
                ctx.fill();

                // Heading arrow
                ctx.strokeStyle = "#FFFFFF";
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(0, 0);
                ctx.lineTo(16, 0);
                ctx.stroke();

                ctx.restore();
            }
        }

        async function updateTelemetry() {
            try {
                const res = await fetch('/telemetry');
                const data = await res.json();

                // Update UI elements
                document.getElementById('cam-status').innerText = data.camera.is_active ? "ACTIVE" : "INACTIVE";
                document.getElementById('cam-status').style.color = data.camera.is_active ? "var(--accent-emerald)" : "var(--accent-rose)";
                document.getElementById('cam-fps').innerText = `${data.camera.actual_fps.toFixed(1)} FPS`;
                document.getElementById('cam-ts').innerText = data.camera.last_timestamp ? `${data.camera.last_timestamp.toFixed(2)}s` : '--';

                document.getElementById('lidar-source').innerText = data.lidar.source;
                document.getElementById('lidar-pts').innerText = data.lidar.num_points;
                document.getElementById('sync-lat').innerText = `${data.digital_twin.pipeline_latency_ms.toFixed(1)} ms`;
                document.getElementById('obs-count').innerText = data.digital_twin.confirmed_obstacle_count;
                document.getElementById('sim-status').innerText = data.simulation.is_connected ? "CONNECTED" : "DETACHED";

                // Robot
                const p = data.robot.pose;
                document.getElementById('robot-pos').innerText = `(${p[0].toFixed(2)}, ${p[1].toFixed(2)})`;
                document.getElementById('robot-yaw').innerText = `${(p[2] * 180 / Math.PI).toFixed(1)}°`;
                document.getElementById('wheel-speeds').innerText = `${data.robot.wheel_speeds[0].toFixed(2)} / ${data.robot.wheel_speeds[1].toFixed(2)}`;
                
                const badge = document.getElementById('nav-state-badge');
                badge.innerText = data.robot.nav_state;
                badge.className = `state-tag state-${data.robot.nav_state}`;

                // Table
                const tbody = document.getElementById('obstacles-tbody');
                if (data.digital_twin.obstacles.length === 0) {
                    tbody.innerHTML = '<tr><td colspan="5" style="text-align: center; color: var(--text-secondary);">No obstacles detected</td></tr>';
                } else {
                    tbody.innerHTML = data.digital_twin.obstacles.map(o => `
                        <tr>
                            <td style="font-family: 'JetBrains Mono'; color: var(--accent-rose); font-weight: 600;">${o.id}</td>
                            <td>[${o.centroid[0].toFixed(2)}, ${o.centroid[1].toFixed(2)}]</td>
                            <td>${(o.radius * 100).toFixed(1)} cm</td>
                            <td>${(o.confidence * 100).toFixed(0)}%</td>
                            <td>${o.age_s.toFixed(1)}s ago</td>
                        </tr>
                    `).join('');
                }

                drawMap(data);
            } catch (e) {
                console.error("Telemetry fetch failed", e);
            }
        }

        async function injectObstacle(x, y) {
            await fetch(`/api/obstacle?action=add&x=${x}&y=${y}`);
        }

        async function clearObstacles() {
            await fetch(`/api/obstacle?action=clear`);
        }

        setInterval(updateTelemetry, 150);
    </script>
</body>
</html>
"""


class DashboardRequestHandler(http.server.BaseHTTPRequestHandler):
    engine: Optional[DigitalTwinEngine] = None

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))

        elif self.path == "/telemetry":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            if self.engine:
                data = self.engine.get_telemetry()
            else:
                data = {}
            self.wfile.write(json.dumps(data).encode("utf-8"))

        elif self.path.startswith("/api/obstacle"):
            # Query format: /api/obstacle?action=add&x=0.0&y=0.0
            from urllib.parse import urlparse, parse_qs
            parsed = urlparse(self.path)
            qs = parse_qs(parsed.query)
            action = qs.get("action", [""])[0]

            if self.engine:
                if action == "add":
                    x = float(qs.get("x", [0.0])[0])
                    y = float(qs.get("y", [0.0])[0])
                    self.engine.lidar.set_synthetic_obstacle("demo_box", x, y, radius=0.20)
                elif action == "clear":
                    self.engine.lidar.remove_synthetic_obstacle("demo_box")

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"status": "ok"}')

        elif self.path == "/video_feed":
            # MJPEG live camera stream
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            while self.engine and self.engine.is_running:
                frame, _, valid = self.engine.camera.get_latest_frame()
                if frame is not None:
                    # Draw timestamp and watermark
                    cv2.putText(frame, f"LIVE CAM - {time.strftime('%H:%M:%S')}", (15, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ret:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpeg.tobytes())
                        self.wfile.write(b"\r\n")
                time.sleep(0.04)

        else:
            self.send_error(404)

    def log_message(self, format, *args):
        # Suppress routine GET logging to keep console clean
        pass


class DashboardServer:
    def __init__(self, engine: DigitalTwinEngine, host: str = "0.0.0.0", port: int = 8085):
        self.engine = engine
        self.host = host
        self.port = port
        self.httpd: Optional[socketserver.TCPServer] = None
        self._thread: Optional[threading.Thread] = None

    def start(self):
        handler_cls = DashboardRequestHandler
        handler_cls.engine = self.engine

        # Enable port reuse
        socketserver.TCPServer.allow_reuse_address = True
        self.httpd = socketserver.TCPServer((self.host, self.port), handler_cls)
        self._thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self._thread.start()
        logger.info(f"Dashboard server running at http://localhost:{self.port}")

    def stop(self):
        if self.httpd:
            self.httpd.shutdown()
            self.httpd.server_close()
        logger.info("Dashboard server stopped.")
