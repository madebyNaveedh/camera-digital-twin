"""
Optional Streamlit Monitoring Dashboard for Real-Time Digital Twin.
Run using: streamlit run dashboard/streamlit_app.py
"""

import time
import streamlit as st
import numpy as np
import requests

st.set_page_config(page_title="Digital Twin Lab Monitor", layout="wide")

st.title("🛰️ Real-Time Digital Twin Laboratory Monitor")
st.markdown("Live perception, synchronization, and autonomous navigation telemetry.")

col1, col2 = st.columns(2)

with col1:
    st.subheader("📷 Live Camera Stream")
    st.markdown("Displays the physical room camera observing obstacles.")
    st.image("http://localhost:8085/video_feed", caption="Camera Feed", use_column_width=True)

with col2:
    st.subheader("📊 System Telemetry")
    telemetry_placeholder = st.empty()

while True:
    try:
        r = requests.get("http://localhost:8085/telemetry", timeout=1.0)
        if r.status_code == 200:
            data = r.json()
            with telemetry_placeholder.container():
                c1, c2, c3 = st.columns(3)
                c1.metric("Navigation State", data["robot"]["nav_state"])
                c2.metric("Active Obstacles", data["digital_twin"]["confirmed_obstacle_count"])
                c3.metric("Sync Latency", f"{data['digital_twin']['pipeline_latency_ms']:.1f} ms")
                
                st.write("**Robot Pose:**", data["robot"]["pose"])
                st.write("**Tracked Obstacles:**", data["digital_twin"]["obstacles"])
    except Exception:
        telemetry_placeholder.warning("Waiting for Digital Twin Engine on localhost:8085...")
    time.sleep(0.5)
