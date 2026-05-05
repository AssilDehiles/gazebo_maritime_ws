#!/usr/bin/env python3
"""Génération de figures publication-quality — Mission AquaRob 4 waypoints."""
import sys
import os
import math
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.io as pio

# ─── Chemins ─────────────────────────────────────────────────────────────────
BAG_PATH  = os.path.expanduser("~/bags/mission_carre")
FIG_DIR   = os.path.expanduser("~/bags/figures_pro")
HTML_DIR  = os.path.join(FIG_DIR, "html")
os.makedirs(FIG_DIR, exist_ok=True)
os.makedirs(HTML_DIR, exist_ok=True)

# ─── Paramètres système ───────────────────────────────────────────────────────
ORIGIN_LAT   = 36.7538
ORIGIN_LON   = 3.0588
COS_LAT      = math.cos(math.radians(ORIGIN_LAT))
WAYPOINTS_XY = [(0.0, 110.0), (222.0, 110.0), (222.0, 0.0), (0.0, 0.0)]
ACCEPT_RADIUS = 2.0

# ─── Palette couleurs professionnelle ────────────────────────────────────────
C_BLUE   = "#1565C0"
C_RED    = "#C62828"
C_GREEN  = "#2E7D32"
C_ORANGE = "#E65100"
C_PURPLE = "#6A1B9A"

# ─── Style global ─────────────────────────────────────────────────────────────
FONT_FAMILY = "Times New Roman, serif"

AXIS_STYLE = dict(
    gridcolor="#E0E0E0", gridwidth=1,
    linecolor="black", linewidth=1,
    title_font=dict(size=14, family=FONT_FAMILY),
    tickfont=dict(size=11, family=FONT_FAMILY),
)

def make_layout(title_text, xlabel, ylabel, width, height, extra_xaxis=None, extra_yaxis=None, **kwargs):
    xax = dict(**AXIS_STYLE, title=xlabel)
    if extra_xaxis:
        xax.update(extra_xaxis)
    yax = dict(**AXIS_STYLE, title=ylabel)
    if extra_yaxis:
        yax.update(extra_yaxis)
    layout = dict(
        template="plotly_white",
        font=dict(family=FONT_FAMILY, size=12, color="black"),
        paper_bgcolor="white",
        plot_bgcolor="white",
        margin=dict(l=80, r=40, t=80, b=60),
        title=dict(text=title_text, font=dict(size=18, family=FONT_FAMILY, color="black")),
        xaxis=xax,
        yaxis=yax,
        legend=dict(font=dict(size=12, family=FONT_FAMILY), bgcolor="rgba(255,255,255,0.9)", bordercolor="#cccccc", borderwidth=1),
        width=width,
        height=height,
    )
    layout.update(kwargs)
    return layout

def save(fig, name):
    png_path  = os.path.join(FIG_DIR,  f"{name}.png")
    html_path = os.path.join(HTML_DIR, f"{name}.html")
    fig.write_image(png_path, scale=300/96)
    fig.write_html(html_path)
    print(f"  PNG  → {png_path}")
    print(f"  HTML → {html_path}")

# ═══════════════════════════════════════════════════════════════════════════════
# LECTURE DU BAG
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{'='*60}")
print(f"  Lecture du bag : {BAG_PATH}")
print(f"{'='*60}")

reader = rosbag2_py.SequentialReader()
storage_options  = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id="mcap")
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)

odom_t, odom_x, odom_y, odom_qz, odom_qw = [], [], [], [], []
left_t, left_f, right_t, right_f          = [], [], [], []
wp_t,   wp_idx                            = [], []

while reader.has_next():
    topic, data, timestamp = reader.read_next()
    t_sec = timestamp * 1e-9
    if topic == "/usv/odometry":
        msg = deserialize_message(data, get_message("nav_msgs/msg/Odometry"))
        odom_t.append(t_sec)
        odom_x.append(msg.pose.pose.position.x)
        odom_y.append(msg.pose.pose.position.y)
        odom_qz.append(msg.pose.pose.orientation.z)
        odom_qw.append(msg.pose.pose.orientation.w)
    elif topic == "/left_thrust_cmd":
        msg = deserialize_message(data, get_message("std_msgs/msg/Float64"))
        left_t.append(t_sec);  left_f.append(msg.data)
    elif topic == "/right_thrust_cmd":
        msg = deserialize_message(data, get_message("std_msgs/msg/Float64"))
        right_t.append(t_sec); right_f.append(msg.data)
    elif topic == "/usv/waypoint_index":
        msg = deserialize_message(data, get_message("std_msgs/msg/Int32"))
        wp_t.append(t_sec);    wp_idx.append(msg.data)

print(f"  Odométrie        : {len(odom_t)} points")
print(f"  Commandes gauche : {len(left_t)} points")
print(f"  Commandes droite : {len(right_t)} points")
print(f"  Waypoint index   : {len(wp_t)} points")

# ─── Normalisation temporelle ─────────────────────────────────────────────────
odom_t  = np.array(odom_t);  t0 = odom_t[0]; odom_t -= t0
odom_x  = np.array(odom_x)
odom_y  = np.array(odom_y)
odom_qz = np.array(odom_qz)
odom_qw = np.array(odom_qw)

left_t  = np.array(left_t);  left_t  -= left_t[0];  left_f  = np.array(left_f)
right_t = np.array(right_t); right_t -= right_t[0]; right_f = np.array(right_f)
wp_t    = np.array(wp_t);    wp_t    -= wp_t[0];    wp_idx  = np.array(wp_idx, dtype=int)

# ─── Calculs dérivés ──────────────────────────────────────────────────────────
cap_deg = (90.0 - np.degrees(2.0 * np.arctan2(odom_qz, odom_qw))) % 360.0

distances  = []
cap_errors = []
for i, (x, y) in enumerate(zip(odom_x, odom_y)):
    t    = odom_t[i]
    idx  = int(np.interp(t, wp_t, wp_idx)) if len(wp_t) > 0 else 0
    idx  = min(idx, len(WAYPOINTS_XY) - 1)
    wx, wy = WAYPOINTS_XY[idx]
    distances.append(math.sqrt((wx - x)**2 + (wy - y)**2))
    desired = math.degrees(math.atan2(wx - x, wy - y)) % 360.0
    err = desired - cap_deg[i]
    if err >  180: err -= 360
    if err < -180: err += 360
    cap_errors.append(err)

distances  = np.array(distances)
cap_errors = np.array(cap_errors)

wp_change_times = [wp_t[i] for i in range(1, len(wp_idx)) if wp_idx[i] != wp_idx[i-1]]
wp_change_labels = ["WP1→WP2", "WP2→WP3", "WP3→WP4"]

# ─── Instants où distance < 2 m ──────────────────────────────────────────────
wp_reached_mask = distances < ACCEPT_RADIUS

print(f"\n  Changements de waypoint à t = {[round(t, 1) for t in wp_change_times]} s")
print(f"  Durée totale mission : {odom_t[-1]:.1f} s")

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 1 — Trajectoire XY
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{'─'*40}")
print("  Figure 1 — Trajectoire XY")

t_norm = odom_t / odom_t[-1]

fig1 = go.Figure()

# Trajectoire avec dégradé bleu→rouge
fig1.add_trace(go.Scatter(
    x=odom_x, y=odom_y,
    mode="markers",
    marker=dict(
        size=4,
        color=odom_t,
        colorscale="RdYlBu_r",
        colorbar=dict(
            title=dict(text="Temps (s)", font=dict(family=FONT_FAMILY, size=13)),
            tickfont=dict(family=FONT_FAMILY, size=11),
            thickness=15,
            len=0.75,
        ),
        showscale=True,
        opacity=0.9,
    ),
    name="Trajectoire USV",
    showlegend=True,
))

# Cercles d'acceptation (2 m) autour de chaque waypoint
theta = np.linspace(0, 2 * np.pi, 80)
for wx, wy in WAYPOINTS_XY:
    fig1.add_trace(go.Scatter(
        x=wx + ACCEPT_RADIUS * np.cos(theta),
        y=wy + ACCEPT_RADIUS * np.sin(theta),
        mode="lines",
        line=dict(color=C_GREEN, width=1.2, dash="dot"),
        fill="toself",
        fillcolor="rgba(46,125,50,0.06)",
        showlegend=False,
        hoverinfo="skip",
    ))

# Waypoints cibles
wp_labels = ["WP1", "WP2", "WP3", "WP4"]
for i, ((wx, wy), lbl) in enumerate(zip(WAYPOINTS_XY, wp_labels)):
    fig1.add_trace(go.Scatter(
        x=[wx], y=[wy],
        mode="markers+text",
        marker=dict(symbol="star", size=20, color=C_RED, line=dict(color="white", width=1)),
        text=[f"  {lbl}"],
        textposition="middle right",
        textfont=dict(size=13, color=C_RED, family=FONT_FAMILY),
        name=lbl,
        showlegend=True,
    ))

# Point de départ
fig1.add_trace(go.Scatter(
    x=[odom_x[0]], y=[odom_y[0]],
    mode="markers",
    marker=dict(symbol="star", size=20, color=C_GREEN, line=dict(color="white", width=1)),
    name="Départ",
    showlegend=True,
))

# Flèches de direction tous les 50 points
arrow_step = max(50, len(odom_x) // 20)
annotations = []
for i in range(arrow_step, len(odom_x) - arrow_step, arrow_step):
    dx = odom_x[i+1] - odom_x[i-1]
    dy = odom_y[i+1] - odom_y[i-1]
    norm = math.sqrt(dx**2 + dy**2) + 1e-9
    scale = 6.0
    annotations.append(dict(
        x=odom_x[i] + scale * dx / norm,
        y=odom_y[i] + scale * dy / norm,
        ax=odom_x[i],
        ay=odom_y[i],
        xref="x", yref="y", axref="x", ayref="y",
        showarrow=True, arrowhead=2, arrowsize=1.5, arrowwidth=1.5,
        arrowcolor=C_BLUE,
    ))

layout1 = make_layout(
    "Trajectoire de navigation — USV AquaRob (Mission 4 waypoints)",
    "Position Est (m)", "Position Nord (m)",
    900, 800,
    extra_xaxis=dict(scaleanchor="y", scaleratio=1),
    annotations=annotations,
)
fig1.update_layout(**layout1)
save(fig1, "figure1_trajectoire_pro")

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 2 — Distance vs temps
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{'─'*40}")
print("  Figure 2 — Distance vs temps")

fig2 = go.Figure()

# Remplissage sous la courbe
fig2.add_trace(go.Scatter(
    x=odom_t, y=distances,
    mode="lines",
    fill="tozeroy",
    fillcolor=f"rgba(21,101,192,0.1)",
    line=dict(color=C_BLUE, width=2.5),
    name="Distance au waypoint courant",
))

# Marqueurs waypoints atteints
if np.any(wp_reached_mask):
    fig2.add_trace(go.Scatter(
        x=odom_t[wp_reached_mask],
        y=distances[wp_reached_mask],
        mode="markers",
        marker=dict(color=C_GREEN, size=5, symbol="circle"),
        name="Waypoint atteint (< 2 m)",
        showlegend=True,
    ))

# Ligne d'acceptation
fig2.add_hline(y=ACCEPT_RADIUS, line=dict(color=C_GREEN, width=2, dash="dash"),
               annotation_text="Rayon d'acceptation (2 m)",
               annotation_font=dict(size=12, family=FONT_FAMILY, color=C_GREEN),
               annotation_position="right")

# Lignes verticales + annotations par segment
seg_labels = ["WP1→WP2", "WP2→WP3", "WP3→WP4", "WP4 atteint"]
t_segments = [0.0] + list(wp_change_times) + [odom_t[-1]]
annotations2 = []

for j, t_change in enumerate(wp_change_times):
    fig2.add_vline(x=t_change, line=dict(color=C_RED, width=1.5, dash="dash"))

for j, label in enumerate(seg_labels):
    t_mid = (t_segments[j] + t_segments[j+1]) / 2.0 if j < len(t_segments)-1 else t_segments[j]
    y_pos = max(distances) * 0.88
    annotations2.append(dict(
        x=t_mid, y=y_pos,
        text=f"<b>{label}</b>",
        showarrow=False,
        font=dict(size=11, color=C_ORANGE, family=FONT_FAMILY),
        bgcolor="rgba(255,255,255,0.75)",
        bordercolor=C_ORANGE,
        borderwidth=1,
    ))

layout2 = make_layout(
    "Convergence vers les waypoints — Distance euclidienne",
    "Temps (s)", "Distance au waypoint (m)",
    1100, 600,
    annotations=annotations2,
)
fig2.update_layout(**layout2)
save(fig2, "figure2_distance_pro")

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 3 — Commandes moteurs vs temps
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{'─'*40}")
print("  Figure 3 — Commandes moteurs")

fig3 = go.Figure()

# Remplissage entre les deux courbes
fig3.add_trace(go.Scatter(
    x=right_t, y=right_f,
    mode="lines",
    line=dict(color=C_RED, width=2.5),
    name="Propulseur droit (FR)",
))
fig3.add_trace(go.Scatter(
    x=left_t, y=left_f,
    mode="lines",
    fill="tonexty",
    fillcolor="rgba(106,27,154,0.1)",
    line=dict(color=C_BLUE, width=2.5),
    name="Propulseur gauche (FL)",
))

# Ligne zéro
fig3.add_hline(y=0, line=dict(color="black", width=1.5))

# Lignes verticales aux changements de waypoint
for t_change in wp_change_times:
    fig3.add_vline(x=t_change, line=dict(color="#999999", width=1.2, dash="dash"))

# Annotations "Phase pivot" et "Phase avance"
annotations3 = []
if len(left_t) > 0 and len(right_t) > 0:
    # Ré-interpoler right_f sur la grille left_t pour comparaison
    right_interp = np.interp(left_t, right_t, right_f)
    diff_abs = np.abs(left_f - right_interp)

    # Segments de pivot : |diff| > 20 N
    pivot_mask = diff_abs > 20.0
    advance_mask = ~pivot_mask

    # Trouver les runs de pivot
    in_pivot = False
    for k in range(len(left_t)):
        if pivot_mask[k] and not in_pivot:
            in_pivot = True
            t_start_pivot = left_t[k]
        elif not pivot_mask[k] and in_pivot:
            in_pivot = False
            t_mid_pivot = (t_start_pivot + left_t[k]) / 2.0
            y_peak = max(abs(left_f[max(0, k-10):k].max() if k > 0 else 0),
                         abs(right_interp[max(0, k-10):k].max() if k > 0 else 0)) + 5
            annotations3.append(dict(
                x=t_mid_pivot,
                y=float(np.max(np.abs(left_f[max(0,k-15):k+1]))) + 5,
                text="Phase pivot",
                showarrow=True, arrowhead=2, arrowcolor=C_PURPLE,
                font=dict(size=10, color=C_PURPLE, family=FONT_FAMILY),
                bgcolor="rgba(255,255,255,0.8)",
            ))

    # Un label "Phase avance" au milieu du premier long segment stable
    stable_run = 0
    best_stable_start = best_stable_len = 0
    run_start = 0
    for k in range(1, len(left_t)):
        if advance_mask[k]:
            stable_run += 1
            if stable_run > best_stable_len:
                best_stable_len = stable_run
                best_stable_start = run_start
        else:
            stable_run = 0
            run_start = k
    if best_stable_len > 5:
        t_adv = left_t[best_stable_start + best_stable_len // 2]
        annotations3.append(dict(
            x=float(t_adv),
            y=float(np.mean(left_f[best_stable_start:best_stable_start+best_stable_len])) + 4,
            text="Phase avance",
            showarrow=False,
            font=dict(size=10, color=C_BLUE, family=FONT_FAMILY),
            bgcolor="rgba(255,255,255,0.8)",
            bordercolor=C_BLUE, borderwidth=1,
        ))

layout3 = make_layout(
    "Commandes différentielles des propulseurs — Contrôleur PID",
    "Temps (s)", "Force de poussée (N)",
    1100, 600,
    annotations=annotations3,
)
fig3.update_layout(**layout3)
save(fig3, "figure3_commandes_pro")

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 4 — Erreur de cap vs temps
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{'─'*40}")
print("  Figure 4 — Erreur de cap")

fig4 = go.Figure()

# Bande orange ±15° (correction active)
fig4.add_trace(go.Scatter(
    x=np.concatenate([odom_t, odom_t[::-1]]),
    y=np.concatenate([np.full_like(odom_t, 15.0), np.full_like(odom_t, -15.0)]),
    fill="toself",
    fillcolor="rgba(230,81,0,0.08)",
    line=dict(color="rgba(0,0,0,0)"),
    name="Zone correction active (±15°)",
    showlegend=True,
    hoverinfo="skip",
))

# Bande verte ±5° (zone acceptable)
fig4.add_trace(go.Scatter(
    x=np.concatenate([odom_t, odom_t[::-1]]),
    y=np.concatenate([np.full_like(odom_t, 5.0), np.full_like(odom_t, -5.0)]),
    fill="toself",
    fillcolor="rgba(46,125,50,0.12)",
    line=dict(color="rgba(0,0,0,0)"),
    name="Zone acceptable (±5°)",
    showlegend=True,
    hoverinfo="skip",
))

# Courbe erreur de cap
fig4.add_trace(go.Scatter(
    x=odom_t, y=cap_errors,
    mode="lines",
    line=dict(color=C_GREEN, width=2.5),
    name="Erreur de cap",
))

# Ligne zéro
fig4.add_hline(y=0, line=dict(color="black", width=1.5, dash="dot"))

# Lignes verticales aux changements de waypoint + annotations rotations
annotations4 = []
rotation_labels = ["Rotation WP1→WP2", "Rotation WP2→WP3", "Rotation WP3→WP4"]
for j, t_change in enumerate(wp_change_times):
    fig4.add_vline(x=t_change, line=dict(color=C_RED, width=1.5, dash="dash"))
    if j < len(rotation_labels):
        # Trouver le pic d'erreur juste après le changement
        mask_after = (odom_t > t_change) & (odom_t < t_change + 20.0)
        if np.any(mask_after):
            peak_idx = np.argmax(np.abs(cap_errors[mask_after]))
            all_idx = np.where(mask_after)[0]
            t_peak = odom_t[all_idx[peak_idx]]
            y_peak = cap_errors[all_idx[peak_idx]]
            annotations4.append(dict(
                x=float(t_peak),
                y=float(y_peak) + (8 if y_peak >= 0 else -8),
                text=f"<b>{rotation_labels[j]}</b>",
                showarrow=True,
                arrowhead=2, arrowcolor=C_RED,
                font=dict(size=10, color=C_RED, family=FONT_FAMILY),
                bgcolor="rgba(255,255,255,0.85)",
                bordercolor=C_RED, borderwidth=1,
            ))

layout4 = make_layout(
    "Erreur de cap — Contrôleur PID (Kp=0.3, Ki=0.001, Kd=0.05)",
    "Temps (s)", "Erreur de cap (°)",
    1100, 600,
    annotations=annotations4,
)
fig4.update_layout(**layout4)
save(fig4, "figure4_cap_pro")

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 5 — Dashboard synthèse 2x2
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{'─'*40}")
print("  Figure 5 — Dashboard synthèse")

fig5 = make_subplots(
    rows=2, cols=2,
    subplot_titles=[
        "Trajectoire XY",
        "Distance au waypoint",
        "Commandes propulseurs",
        "Erreur de cap",
    ],
    specs=[
        [{"type": "scatter"}, {"type": "scatter"}],
        [{"type": "scatter"}, {"type": "scatter"}],
    ],
    horizontal_spacing=0.10,
    vertical_spacing=0.12,
)

# ── Subplot 1 : Trajectoire ───────────────────────────────────────────────────
fig5.add_trace(go.Scatter(
    x=odom_x, y=odom_y,
    mode="markers",
    marker=dict(size=3, color=odom_t, colorscale="RdYlBu_r", showscale=False),
    showlegend=False,
), row=1, col=1)
for (wx, wy), lbl in zip(WAYPOINTS_XY, wp_labels):
    fig5.add_trace(go.Scatter(
        x=[wx], y=[wy],
        mode="markers",
        marker=dict(symbol="star", size=10, color=C_RED),
        showlegend=False,
    ), row=1, col=1)
fig5.add_trace(go.Scatter(
    x=[odom_x[0]], y=[odom_y[0]],
    mode="markers",
    marker=dict(symbol="star", size=10, color=C_GREEN),
    showlegend=False,
), row=1, col=1)

# ── Subplot 2 : Distance ──────────────────────────────────────────────────────
fig5.add_trace(go.Scatter(
    x=odom_t, y=distances,
    mode="lines",
    line=dict(color=C_BLUE, width=2),
    showlegend=False,
), row=1, col=2)
fig5.add_hline(y=ACCEPT_RADIUS, line=dict(color=C_GREEN, width=1.5, dash="dash"), row=1, col=2)
for t_change in wp_change_times:
    fig5.add_vline(x=t_change, line=dict(color=C_RED, width=1, dash="dash"), row=1, col=2)

# ── Subplot 3 : Commandes ─────────────────────────────────────────────────────
fig5.add_trace(go.Scatter(
    x=left_t, y=left_f,
    mode="lines",
    line=dict(color=C_BLUE, width=2),
    showlegend=False,
), row=2, col=1)
fig5.add_trace(go.Scatter(
    x=right_t, y=right_f,
    mode="lines",
    line=dict(color=C_RED, width=2),
    showlegend=False,
), row=2, col=1)
for t_change in wp_change_times:
    fig5.add_vline(x=t_change, line=dict(color="#999999", width=1, dash="dash"), row=2, col=1)

# ── Subplot 4 : Erreur de cap ─────────────────────────────────────────────────
fig5.add_trace(go.Scatter(
    x=odom_t, y=cap_errors,
    mode="lines",
    line=dict(color=C_GREEN, width=2),
    showlegend=False,
), row=2, col=2)
fig5.add_hline(y=0, line=dict(color="black", width=1, dash="dot"), row=2, col=2)
for t_change in wp_change_times:
    fig5.add_vline(x=t_change, line=dict(color=C_RED, width=1, dash="dash"), row=2, col=2)

fig5.update_layout(
    title=dict(
        text="Synthèse des performances — Mission AquaRob 4 waypoints",
        font=dict(size=20, family=FONT_FAMILY, color="black"),
        x=0.5, xanchor="center",
    ),
    template="plotly_white",
    font=dict(family=FONT_FAMILY, size=11),
    paper_bgcolor="white",
    plot_bgcolor="white",
    margin=dict(l=70, r=40, t=100, b=60),
    width=1400, height=1000,
)
fig5.update_xaxes(gridcolor="#E0E0E0", linecolor="black", linewidth=1)
fig5.update_yaxes(gridcolor="#E0E0E0", linecolor="black", linewidth=1)
fig5.update_xaxes(title_text="Est (m)",   title_font=dict(size=12), row=1, col=1)
fig5.update_yaxes(title_text="Nord (m)",  title_font=dict(size=12), row=1, col=1)
fig5.update_xaxes(title_text="Temps (s)", title_font=dict(size=12), row=1, col=2)
fig5.update_yaxes(title_text="Distance (m)", title_font=dict(size=12), row=1, col=2)
fig5.update_xaxes(title_text="Temps (s)", title_font=dict(size=12), row=2, col=1)
fig5.update_yaxes(title_text="Force (N)", title_font=dict(size=12), row=2, col=1)
fig5.update_xaxes(title_text="Temps (s)", title_font=dict(size=12), row=2, col=2)
fig5.update_yaxes(title_text="Erreur (°)", title_font=dict(size=12), row=2, col=2)

save(fig5, "figure5_dashboard_pro")

# ═══════════════════════════════════════════════════════════════════════════════
# RÉSUMÉ
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{'='*60}")
print("  RÉSUMÉ — Fichiers générés")
print(f"{'='*60}")
for f in sorted(os.listdir(FIG_DIR)):
    full = os.path.join(FIG_DIR, f)
    if os.path.isfile(full):
        size_kb = os.path.getsize(full) / 1024
        print(f"  {f:45s} {size_kb:7.1f} Ko")
print()
for f in sorted(os.listdir(HTML_DIR)):
    full = os.path.join(HTML_DIR, f)
    if os.path.isfile(full):
        size_kb = os.path.getsize(full) / 1024
        print(f"  html/{f:41s} {size_kb:7.1f} Ko")
print(f"\n  Répertoire PNG  : {FIG_DIR}")
print(f"  Répertoire HTML : {HTML_DIR}")
print(f"{'='*60}\n")
