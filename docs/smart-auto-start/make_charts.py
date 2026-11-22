"""
Makes the smart auto start charts for the README from the analyzer's CSV export. No libraries needed.

    ./gradlew simulateJava -PscriptedSim=autooffset
    ./gradlew analyzeLog -Pexport=docs/smart-auto-start/vision
    ./gradlew simulateJava -PscriptedSim=autonovision
    ./gradlew analyzeLog -Pexport=docs/smart-auto-start/no-vision
    python docs/smart-auto-start/make_charts.py

Writes a light and a dark version of each chart (the README picks one to match the viewer's GitHub theme).
"""

import csv
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# reference palette (categorical slots 1-3, which pass the color checks all-pairs in both modes) and neutrals
THEMES = {
    "light": dict(surface="#fcfcfb", text="#0b0b0b", text2="#52514e", muted="#898781", grid="#e1e0d9", axis="#c3c2b7",
                  series=["#2a78d6", "#eb6834", "#1baf7a"]),
    "dark": dict(surface="#1a1a19", text="#f0efec", text2="#c3c2b7", muted="#898781", grid="#2c2c2a", axis="#383835",
                 series=["#3987e5", "#d95926", "#199e70"]),
}
FONT = "system-ui, -apple-system, 'Segoe UI', Roboto, sans-serif"


def load(folder):
    runs = {}
    with open(os.path.join(HERE, folder, "auto_tracking.csv"), newline="") as f:
        for row in csv.DictReader(f):
            runs.setdefault(row["auto"], []).append((float(row["seconds"]), float(row["error_m"]) * 100))
    return runs


def segments(points, max_gap=0.1):
    """split a run where samples are missing (the analyzer drops the 2 s after a PC freeze) so gaps show as gaps"""
    out, current = [], []
    for p in points:
        if current and p[0] - current[-1][0] > max_gap:
            out.append(current)
            current = []
        current.append(p)
    if current:
        out.append(current)
    return out


def panel(theme, x0, y0, w, h, series, x_max, y_max, x_step, y_step, title, show_y_labels):
    """one chart panel: hairline grid, axis labels, 2px lines, end markers with a surface ring, direct labels"""
    t = THEMES[theme]
    sx = lambda x: x0 + x / x_max * w
    sy = lambda y: y0 + h - min(y, y_max) / y_max * h
    svg = [f'<text x="{x0}" y="{y0 - 12}" font-size="13" font-weight="600" fill="{t["text"]}">{title}</text>']

    y = 0
    while y <= y_max + 1e-9:  # horizontal gridlines + y labels
        color = t["axis"] if y == 0 else t["grid"]
        svg.append(f'<line x1="{x0}" x2="{x0 + w}" y1="{sy(y):.1f}" y2="{sy(y):.1f}" stroke="{color}" stroke-width="1"/>')
        if show_y_labels:
            svg.append(f'<text x="{x0 - 8}" y="{sy(y) + 4:.1f}" font-size="11" fill="{t["muted"]}" text-anchor="end">{y:g}</text>')
        y += y_step
    x = 0
    while x <= x_max + 1e-9:  # x labels
        svg.append(f'<text x="{sx(x):.1f}" y="{y0 + h + 16}" font-size="11" fill="{t["muted"]}" text-anchor="middle">{x:g}</text>')
        x += x_step

    labels = []
    for i, (name, points) in enumerate(series):
        color = t["series"][i]
        points = [p for p in points if p[0] <= x_max]
        for seg in segments(points):
            d = " ".join(f'{"M" if j == 0 else "L"}{sx(px):.1f},{sy(py):.1f}' for j, (px, py) in enumerate(seg))
            svg.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>')
        last = points[-1]
        svg.append(f'<circle cx="{sx(last[0]):.1f}" cy="{sy(last[1]):.1f}" r="4" fill="{color}" stroke="{t["surface"]}" stroke-width="2"/>')
        labels.append([sy(last[1]), name, color])

    # direct labels at the line ends, nudged apart so they never overlap
    labels.sort()
    for j in range(1, len(labels)):
        labels[j][0] = max(labels[j][0], labels[j - 1][0] + 15)
    for ly, name, color in labels:
        lx = x0 + w + 10
        svg.append(f'<line x1="{lx}" x2="{lx + 10}" y1="{ly:.1f}" y2="{ly:.1f}" stroke="{color}" stroke-width="2" stroke-linecap="round"/>')
        svg.append(f'<text x="{lx + 14}" y="{ly + 4:.1f}" font-size="11" fill="{t["text2"]}">{name}</text>')
    return svg


def legend(theme, x, y, names):
    t = THEMES[theme]
    out = []
    for i, name in enumerate(names):
        out.append(f'<line x1="{x}" x2="{x + 16}" y1="{y}" y2="{y}" stroke="{t["series"][i]}" stroke-width="2" stroke-linecap="round"/>')
        out.append(f'<text x="{x + 22}" y="{y + 4}" font-size="12" fill="{t["text2"]}">{name}</text>')
        x += 32 + 7 * len(name)
    return out


def write(name, theme, width, height, body, footnote):
    t = THEMES[theme]
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" font-family="{FONT}">',
           f'<rect width="{width}" height="{height}" rx="8" fill="{t["surface"]}"/>']
    svg += body
    svg.append(f'<text x="56" y="{height - 30}" font-size="11" fill="{t["muted"]}">seconds since auto started</text>')
    svg.append(f'<text x="56" y="{height - 12}" font-size="11" fill="{t["muted"]}">{footnote}</text>')
    svg.append("</svg>")
    with open(os.path.join(HERE, f"{name}-{theme}.svg"), "w", encoding="utf-8") as f:
        f.write("\n".join(svg))


def main():
    vision = load("vision")
    no_vision = load("no-vision")
    gap_note = "gaps: the sim froze on the PC, so those samples were dropped"

    for theme in THEMES:
        # chart 1: with vision, smart start vs. the old reset, from two placements (small multiples, shared scale)
        body = legend(theme, 56, 26, ["Smart start", "Old: reset odometry"])
        body.append(f'<text x="16" y="66" font-size="11" fill="{THEMES[theme]["muted"]}">cm off path</text>')
        for k, (title, smart, old) in enumerate([
            ("20 cm right, turned 5°  (smart start trusts vision)", "auto: 20 cm right, 5 deg", "auto: 20 cm right, 5 deg, OLD"),
            ("45 cm back  (smart start drives to the start first)", "auto: 45 cm back", "auto: 45 cm back, OLD"),
        ]):
            body += panel(theme, 56 + k * 400, 96, 300, 200, [("Smart", vision[smart]), ("Old", vision[old])],
                          x_max=3, y_max=50, x_step=0.5, y_step=10, title=title, show_y_labels=True)
        write("with-vision", theme, 820, 370, body, gap_note)

        # chart 2: no vision, smart start falls back to the old reset
        body = legend(theme, 56, 26, ["On start", "20 cm right, 5°", "45 cm back"])
        body.append(f'<text x="16" y="66" font-size="11" fill="{THEMES[theme]["muted"]}">cm off path</text>')
        body += panel(theme, 56, 96, 620, 200, [("On start", no_vision["auto: no vision, on start"]),
                                                ("20 cm right", no_vision["auto: no vision, 20 cm right, 5 deg"]),
                                                ("45 cm back", no_vision["auto: no vision, 45 cm back"])],
                      x_max=5, y_max=70, x_step=0.5, y_step=10, title="No vision: every placement uses the old reset", show_y_labels=True)
        write("no-vision", theme, 820, 370, body, gap_note)
    print("wrote charts to", HERE)


if __name__ == "__main__":
    main()
