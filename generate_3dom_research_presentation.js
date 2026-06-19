const pptxgen = require("pptxgenjs");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Ismaele Carbonari";
pptx.company = "University of Trento";
pptx.subject = "Research discussion with FBK 3DOM";
pptx.title = "From Sim-to-Real Navigation to Future Robotics Research Directions";
pptx.lang = "en-US";
pptx.theme = {
  headFontFace: "Liberation Sans",
  bodyFontFace: "Liberation Sans",
  lang: "en-US",
};
pptx.defineSlideMaster({
  title: "RESEARCH",
  background: { color: "FFFFFF" },
  objects: [
    {
      line: {
        x: 0.55,
        y: 0.25,
        w: 12.23,
        h: 0,
        line: { color: "DCE5EE", width: 0.8 },
      },
    },
    {
      text: {
        text: "ISMAELE CARBONARI  ·  FBK 3DOM",
        options: {
          x: 0.58,
          y: 0.06,
          w: 4.6,
          h: 0.16,
          fontFace: "Liberation Sans",
          fontSize: 7.5,
          bold: true,
          color: "50657A",
          charSpacing: 1.1,
          margin: 0,
        },
      },
    },
  ],
  slideNumber: {
    x: 12.3,
    y: 7.15,
    w: 0.45,
    h: 0.18,
    color: "6F8193",
    fontFace: "Liberation Sans",
    fontSize: 8,
    align: "right",
    margin: 0,
  },
});

const C = {
  navy: "0B2341",
  blue: "1667A8",
  cyan: "2C9AB7",
  sky: "DDEEF7",
  pale: "F3F7FA",
  ink: "122235",
  muted: "53677A",
  light: "DCE5EE",
  green: "2B8A6E",
  greenLight: "DFF1EB",
  orange: "D9772A",
  orangeLight: "F9E9DA",
  red: "B84949",
  white: "FFFFFF",
};

const P = "/home/isma/Desktop/thesis/photos_videos/photos & videos";
const A = {
  car: `${P}/DSC_0040.JPG`,
  tracked: `${P}/DSC_0050.JPG`,
  gui: `${P}/Screenshot From 2026-05-21 03-42-53.png`,
  telemetry: `${P}/Screenshot From 2026-05-21 03-30-38.png`,
  hardwareMixed: `${P}/hardware mixed.JPG`,
  simulationMixed: `${P}/simulation mixed car.png`,
  unstructured: `${P}/unstructured car.png`,
};
const G = {
  tracking:
    "/home/isma/Desktop/Autonomus-Agent/presentation_assets/trajectory-error-p95-m.png",
};

const OUT =
  "/home/isma/Desktop/Autonomus-Agent/From_Sim_to_Real_Navigation_to_Future_Robotics_Research_Directions.pptx";

function addText(slide, text, x, y, w, h, options = {}) {
  slide.addText(text, {
    x,
    y,
    w,
    h,
    fontFace: "Liberation Sans",
    fontSize: 16,
    color: C.ink,
    margin: 0,
    breakLine: false,
    valign: "mid",
    fit: "shrink",
    ...options,
  });
}

function addTitle(slide, kicker, title, subtitle = "") {
  addText(slide, kicker.toUpperCase(), 0.62, 0.43, 3.8, 0.22, {
    fontSize: 8.5,
    bold: true,
    color: C.blue,
    charSpacing: 1.4,
  });
  addText(slide, title, 0.62, 0.69, 11.9, 0.52, {
    fontSize: 27,
    bold: true,
    color: C.navy,
    valign: "top",
  });
  if (subtitle) {
    addText(slide, subtitle, 0.64, 1.17, 11.6, 0.34, {
      fontSize: 11.5,
      color: C.muted,
      valign: "top",
    });
  }
}

function addImage(slide, path, x, y, w, h, options = {}) {
  // PptxGenJS' cover/contain sizing preserves the source aspect ratio.
  // All photographic frames use cover (crop), while plots use contain.
  slide.addImage({
    path,
    x,
    y,
    w,
    h,
    sizing: { type: options.contain ? "contain" : "cover", w, h },
    transparency: options.transparency || 0,
    altText: options.altText || "",
  });
  if (options.border !== false) {
    slide.addShape(pptx.ShapeType.rect, {
      x,
      y,
      w,
      h,
      fill: { color: "FFFFFF", transparency: 100 },
      line: { color: options.borderColor || C.light, width: options.borderWidth || 1 },
    });
  }
}

function addTag(slide, text, x, y, w, color = C.blue, fill = C.sky) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x,
    y,
    w,
    h: 0.34,
    rectRadius: 0.05,
    fill: { color: fill },
    line: { color: fill, transparency: 100 },
  });
  addText(slide, text, x + 0.1, y + 0.03, w - 0.2, 0.26, {
    fontSize: 9,
    bold: true,
    align: "center",
    color,
  });
}

function addRule(slide, x, y, w, color = C.blue, width = 2) {
  slide.addShape(pptx.ShapeType.line, {
    x,
    y,
    w,
    h: 0,
    line: { color, width },
  });
}

function addArrow(slide, x, y, w, h, color = C.blue, width = 2.2) {
  slide.addShape(pptx.ShapeType.line, {
    x,
    y,
    w,
    h,
    line: { color, width, endArrowType: "triangle" },
  });
}

function addNode(slide, x, y, w, h, title, subtitle, style = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x,
    y,
    w,
    h,
    rectRadius: 0.06,
    fill: { color: style.fill || C.white },
    line: { color: style.line || C.light, width: style.lineWidth || 1.2 },
  });
  addText(slide, title, x + 0.14, y + 0.12, w - 0.28, subtitle ? 0.28 : h - 0.24, {
    fontSize: style.titleSize || 14,
    bold: true,
    color: style.titleColor || C.navy,
    align: style.align || "center",
  });
  if (subtitle) {
    addText(slide, subtitle, x + 0.14, y + 0.45, w - 0.28, h - 0.55, {
      fontSize: style.subtitleSize || 9.5,
      color: style.subtitleColor || C.muted,
      align: style.align || "center",
      valign: "top",
    });
  }
}

function addBulletLines(slide, items, x, y, w, lineHeight = 0.42, color = C.ink) {
  items.forEach((item, i) => {
    slide.addShape(pptx.ShapeType.ellipse, {
      x,
      y: y + i * lineHeight + 0.13,
      w: 0.08,
      h: 0.08,
      fill: { color: C.blue },
      line: { color: C.blue, transparency: 100 },
    });
    addText(slide, item, x + 0.18, y + i * lineHeight, w - 0.18, 0.34, {
      fontSize: 13,
      color,
    });
  });
}

function addPlaceholder(slide, type, x, y, w, h, label, sublabel) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x,
    y,
    w,
    h,
    rectRadius: 0.06,
    fill: { color: C.pale },
    line: { color: "A6B8C8", width: 1.2, dash: "dash" },
  });

  const cx = x + w / 2;
  const cy = y + h * 0.43;
  if (type === "rover") {
    slide.addShape(pptx.ShapeType.roundRect, {
      x: cx - 0.65,
      y: cy - 0.3,
      w: 1.3,
      h: 0.48,
      rectRadius: 0.04,
      fill: { color: C.sky },
      line: { color: C.blue, width: 1.5 },
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: cx - 0.27,
      y: cy - 0.58,
      w: 0.54,
      h: 0.28,
      fill: { color: C.white },
      line: { color: C.blue, width: 1.3 },
    });
    [-0.46, 0.46].forEach((dx) => {
      slide.addShape(pptx.ShapeType.ellipse, {
        x: cx + dx - 0.15,
        y: cy + 0.08,
        w: 0.3,
        h: 0.3,
        fill: { color: C.navy },
        line: { color: C.navy },
      });
    });
  } else {
    slide.addShape(pptx.ShapeType.ellipse, {
      x: cx - 0.18,
      y: cy - 0.18,
      w: 0.36,
      h: 0.36,
      fill: { color: C.blue },
      line: { color: C.blue },
    });
    [
      [-0.78, -0.5],
      [0.78, -0.5],
      [-0.78, 0.5],
      [0.78, 0.5],
    ].forEach(([dx, dy]) => {
      addRule(slide, cx + Math.sign(dx) * 0.12, cy + Math.sign(dy) * 0.1, dx * 0.58, C.blue, 1.5);
      slide.addShape(pptx.ShapeType.ellipse, {
        x: cx + dx - 0.19,
        y: cy + dy - 0.19,
        w: 0.38,
        h: 0.38,
        fill: { color: C.white, transparency: 100 },
        line: { color: C.blue, width: 1.4 },
      });
    });
  }
  addText(slide, label.toUpperCase(), x + 0.3, y + h - 0.72, w - 0.6, 0.2, {
    fontSize: 8,
    bold: true,
    charSpacing: 1.2,
    align: "center",
    color: C.blue,
  });
  addText(slide, sublabel, x + 0.25, y + h - 0.46, w - 0.5, 0.25, {
    fontSize: 10,
    align: "center",
    color: C.muted,
  });
}

function addNotes(slide, timing, visual, notes) {
  slide.addNotes(
    `Timing: ${timing}\nVisual cue: ${visual}\n\nSpeaker notes:\n${notes}`
  );
}

// Slide 1 — Title
{
  const slide = pptx.addSlide();
  slide.background = { color: C.white };
  addImage(slide, A.gui, 7.25, 0, 6.08, 7.5, {
    border: false,
    altText: "Thesis simulator showing LiDAR gates and navigation state",
  });
  slide.addShape(pptx.ShapeType.rect, {
    x: 7.25,
    y: 0,
    w: 6.08,
    h: 7.5,
    fill: { color: C.navy, transparency: 52 },
    line: { color: C.navy, transparency: 100 },
  });
  slide.addShape(pptx.ShapeType.rect, {
    x: 6.93,
    y: 0,
    w: 0.32,
    h: 7.5,
    fill: { color: C.blue },
    line: { color: C.blue },
  });
  addText(slide, "RESEARCH DISCUSSION", 0.72, 0.78, 4.3, 0.24, {
    fontSize: 10,
    bold: true,
    color: C.blue,
    charSpacing: 1.8,
  });
  addText(
    slide,
    "From Sim-to-Real\nNavigation to Future\nRobotics Research\nDirections",
    0.72,
    1.24,
    5.9,
    3.2,
    {
      fontSize: 29,
      bold: true,
      color: C.navy,
      breakLine: true,
      valign: "top",
      paraSpaceAfterPt: 7,
    }
  );
  addRule(slide, 0.74, 4.72, 1.15, C.cyan, 4);
  addText(slide, "Ismaele Carbonari", 0.72, 5.08, 4.5, 0.35, {
    fontSize: 16,
    bold: true,
    color: C.ink,
  });
  addText(slide, "University of Trento", 0.72, 5.45, 4.5, 0.28, {
    fontSize: 12,
    color: C.muted,
  });
  addText(slide, "Meeting with FBK 3DOM", 0.72, 6.25, 4.5, 0.28, {
    fontSize: 12,
    bold: true,
    color: C.blue,
  });
  addText(slide, "June 2026", 0.72, 6.59, 2.5, 0.25, {
    fontSize: 10,
    color: C.muted,
  });
  addNotes(
    slide,
    "0:30",
    "Open on the complete simulator, immediately framing the work as an integrated research system.",
    "Thank you for the opportunity to discuss my work. I will briefly present the navigation and validation framework developed for my BSc thesis, then use it as a starting point for possible collaboration directions with 3DOM. The central idea is transferability: the result is not tied to one robot, but to a modular framework that preserves the high-level planner while adapting perception, estimation and actuation to each platform."
  );
}

// Slide 2 — About Me
{
  const slide = pptx.addSlide("RESEARCH");
  addTitle(slide, "Background", "About me", "Building robotic systems from software architecture to physical deployment");
  addImage(slide, A.car, 0.65, 1.7, 4.05, 3.05, {
    altText: "Four-wheel robot car developed for the thesis",
  });
  addImage(slide, A.tracked, 4.89, 1.7, 3.28, 3.05, {
    altText: "Tracked robot used for portability testing",
  });
  addText(slide, "BSc Computer Science", 8.65, 1.7, 3.8, 0.35, {
    fontSize: 20,
    bold: true,
    color: C.navy,
  });
  addText(slide, "University of Trento", 8.65, 2.09, 3.8, 0.27, {
    fontSize: 12,
    color: C.blue,
    bold: true,
  });
  addRule(slide, 8.65, 2.53, 3.5, C.light, 1);
  addBulletLines(
    slide,
    [
      "Autonomous navigation",
      "State estimation",
      "Embedded systems",
      "Real robot deployment",
    ],
    8.67,
    2.73,
    3.75,
    0.5
  );
  addText(slide, "PATH TO THE CURRENT WORK", 0.67, 5.18, 2.8, 0.2, {
    fontSize: 8.5,
    bold: true,
    color: C.muted,
    charSpacing: 1.4,
  });
  const timeline = [
    ["2024", "Personal robotics\nprojects"],
    ["2025", "Autonomous navigation\nprototype"],
    ["2026", "Sim-to-real BSc thesis\n+ tracked extension"],
    ["NEXT", "Research\ncollaboration"],
  ];
  timeline.forEach(([year, label], i) => {
    const x = 0.72 + i * 3.05;
    if (i < timeline.length - 1) addArrow(slide, x + 2.5, 5.94, 0.42, 0, C.light, 1.5);
    slide.addShape(pptx.ShapeType.ellipse, {
      x,
      y: 5.67,
      w: 0.56,
      h: 0.56,
      fill: { color: i === timeline.length - 1 ? C.orange : C.blue },
      line: { color: i === timeline.length - 1 ? C.orange : C.blue },
    });
    addText(slide, year, x, 5.76, 0.56, 0.3, {
      fontSize: year === "NEXT" ? 7.5 : 9,
      bold: true,
      color: C.white,
      align: "center",
    });
    addText(slide, label, x + 0.72, 5.62, 1.8, 0.68, {
      fontSize: 11,
      bold: true,
      color: C.navy,
      valign: "top",
    });
  });
  addText(slide, "C/C++ · embedded systems · LiDAR · SLAM · state estimation · real deployment", 0.72, 6.62, 11.55, 0.28, {
    fontSize: 11,
    bold: true,
    color: C.blue,
    align: "center",
  });
  addNotes(
    slide,
    "1:00",
    "Use the two robots as evidence of hands-on work, then move quickly across the timeline.",
    "My background is in computer science, but my strongest interest is at the boundary between algorithms and physical systems. Personal robotics projects led to an autonomous navigation prototype and then to the sim-to-real thesis, including the tracked robot extension. Across that progression I worked with C and C++, embedded firmware, LiDAR sensing, state estimation, communication and real deployment. The next step I am exploring is research collaboration."
  );
}

// Slide 3 — Motivation
{
  const slide = pptx.addSlide("RESEARCH");
  addTitle(
    slide,
    "Thesis motivation",
    "A planner that works in simulation is not yet a robotic system",
    "Research question: how can the same navigation framework be transferred to real, heterogeneous platforms?"
  );
  addImage(slide, A.simulationMixed, 0.65, 1.75, 4.3, 3.12, {
    altText: "Mixed navigation scenario in simulation",
  });
  addImage(slide, A.hardwareMixed, 8.38, 1.75, 4.3, 3.12, {
    altText: "Hardware navigation observed through the same interface",
  });
  addText(slide, "SIMULATION", 0.68, 5.02, 2, 0.22, {
    fontSize: 9,
    bold: true,
    color: C.blue,
    charSpacing: 1.2,
  });
  addText(slide, "PHYSICAL SYSTEM", 10.55, 5.02, 2.1, 0.22, {
    fontSize: 9,
    bold: true,
    color: C.orange,
    align: "right",
    charSpacing: 1.2,
  });
  addArrow(slide, 5.22, 3.27, 2.83, 0, C.navy, 3.2);
  addText(slide, "MEASURE THE GAP", 5.24, 2.74, 2.77, 0.28, {
    fontSize: 11,
    bold: true,
    align: "center",
    color: C.navy,
  });
  [
    ["Sensor noise", 0.78],
    ["Wheel slip", 2.7],
    ["Actuator saturation", 4.25],
    ["Communication delay", 6.55],
    ["Estimation error", 8.92],
    ["Vehicle kinematics", 10.72],
  ].forEach(([t, x], i) =>
    addTag(
      slide,
      t,
      x,
      5.58,
      i === 2 || i === 3 ? 2.05 : 1.58,
      i < 3 ? C.blue : C.orange,
      i < 3 ? C.sky : C.orangeLight
    )
  );
  addText(
    slide,
    "Goal: build an experimental infrastructure that makes the sim-to-real gap observable and quantitatively comparable.",
    0.7,
    6.43,
    11.9,
    0.45,
    {
      fontSize: 17,
      bold: true,
      align: "center",
      color: C.navy,
    }
  );
  addNotes(
    slide,
    "1:00",
    "The visual bridge is the thesis problem: the same task is seen once in simulation and once through hardware telemetry.",
    "The thesis started from a practical question: what must be added around a planner before it can operate on a real robot? Simulation hides several effects that become dominant on hardware, including slip, motor asymmetry, sensor noise, delays and model mismatch. My objective was therefore not to force simulation and hardware to look identical, but to make their differences measurable and attributable to specific layers of the system."
  );
}

// Slide 4 — Architecture
{
  const slide = pptx.addSlide("RESEARCH");
  addTitle(
    slide,
    "System architecture",
    "One decision layer, multiple platform-dependent interfaces",
    "The planner operates on geometric abstractions; sensing and actuation adapt to the robot."
  );

  slide.addShape(pptx.ShapeType.roundRect, {
    x: 0.52,
    y: 1.78,
    w: 4.1,
    h: 3.15,
    rectRadius: 0.04,
    fill: { color: C.sky, transparency: 58 },
    line: { color: C.cyan, width: 1, dash: "dash" },
  });
  slide.addShape(pptx.ShapeType.roundRect, {
    x: 6.9,
    y: 1.78,
    w: 5.95,
    h: 3.15,
    rectRadius: 0.04,
    fill: { color: C.pale, transparency: 24 },
    line: { color: C.blue, width: 1, dash: "dash" },
  });
  addText(slide, "PLATFORM ADAPTATION", 0.72, 1.82, 3.66, 0.2, {
    fontSize: 8,
    bold: true,
    color: C.cyan,
    align: "center",
    charSpacing: 1,
  });
  addText(slide, "PLATFORM ADAPTATION", 7.12, 1.82, 5.5, 0.2, {
    fontSize: 8,
    bold: true,
    color: C.blue,
    align: "center",
    charSpacing: 1,
  });
  [["LiDAR", 2.02], ["IMU", 2.96], ["Encoders", 3.9]].forEach(([name, yy]) => {
    addNode(slide, 0.66, yy, 1.52, 0.58, name, "", {
      fill: C.pale,
      line: C.cyan,
      titleSize: 12.5,
    });
  });

  const y = 2.72;
  const nodes = [
    { x: 2.92, w: 1.52, t: "EKF", s: "state estimate", fill: C.sky, line: C.cyan, label: "FUSION" },
    { x: 4.94, w: 1.68, t: "Motion\nPlanner", s: "decision layer", fill: C.navy, line: C.navy, tc: C.white, sc: "DDEBF5", label: "DECISION" },
    { x: 7.12, w: 1.6, t: "Local\nTracker", s: "local reference", fill: C.pale, line: C.blue, label: "TRACKING" },
    { x: 9.22, w: 1.64, t: "Low-Level\nController", s: "speed / yaw-rate", fill: C.pale, line: C.blue, label: "ACTUATION" },
    { x: 11.36, w: 1.32, t: "Robot", s: "physical platform", fill: C.orangeLight, line: C.orange, label: "PLATFORM" },
  ];
  nodes.forEach((n, i) => {
    addText(slide, n.label, n.x, 2.16, n.w, 0.2, {
      fontSize: 8,
      bold: true,
      color: n.line,
      align: "center",
      charSpacing: 0.8,
    });
    addNode(slide, n.x, y, n.w, 1.28, n.t, n.s, {
      fill: n.fill,
      line: n.line,
      titleColor: n.tc || C.navy,
      subtitleColor: n.sc || C.muted,
      titleSize: 15,
      subtitleSize: 8.5,
    });
    if (i < nodes.length - 1) {
      const next = nodes[i + 1];
      addArrow(slide, n.x + n.w + 0.08, y + 0.64, next.x - (n.x + n.w) - 0.16, 0, C.blue, 2.5);
    }
  });

  addArrow(slide, 2.2, 2.31, 0.66, 0.72, C.cyan, 2.2);
  addArrow(slide, 2.2, 3.25, 0.66, 0.1, C.cyan, 2.2);
  addArrow(slide, 2.2, 4.19, 0.66, -0.72, C.cyan, 2.2);

  addText(slide, "PLANNER CORE · PLATFORM INDEPENDENT", 4.72, 4.27, 2.12, 0.2, {
    fontSize: 8,
    bold: true,
    color: C.navy,
    align: "center",
    charSpacing: 0.8,
  });
  addRule(slide, 4.96, 4.56, 1.62, C.navy, 3);

  slide.addShape(pptx.ShapeType.line, {
    x: 12.02,
    y: 4.92,
    w: -10.48,
    h: 0.93,
    line: { color: C.orange, width: 1.8, dash: "dash", endArrowType: "triangle" },
  });
  addText(slide, "feedback to sensing and state estimation", 4.45, 5.54, 4.55, 0.25, {
    fontSize: 10,
    color: C.muted,
    align: "center",
    italic: true,
  });
  addText(
    slide,
    "The planner remains unchanged while sensing, tracking and actuation adapt to the platform.",
    2.05,
    6.28,
    9.3,
    0.38,
    {
      fontSize: 18,
      bold: true,
      color: C.navy,
      align: "center",
    }
  );
  addNotes(
    slide,
    "1:15",
    "Keep the audience on the dark planner block, then move outward to the adapters and feedback loop.",
    "This is the architecture that organizes the work. LiDAR, IMU and encoders are fused into a state estimate. The planner selects a motion primitive and produces a local reference. A tracker converts that reference into speed and yaw-rate targets, while the low-level layer maps them to the physical actuators. The planner is deliberately isolated from platform details. The feedback path closes the loop through measurements and telemetry, and it is also what makes experimental validation possible."
  );
}

// Slide 5 — Contributions
{
  const slide = pptx.addSlide("RESEARCH");
  addTitle(slide, "Thesis contributions", "What I developed", "A complete experimental stack around an existing general planner");

  const cols = [
    {
      x: 0.64,
      title: "Simulation",
      accent: C.cyan,
      img: A.gui,
      items: ["Custom C++ simulator", "Synthetic sensors", "GUI, telemetry and reports"],
    },
    {
      x: 4.72,
      title: "Real robot integration",
      accent: C.orange,
      img: A.car,
      items: ["Firmware and serial protocol", "EKF and hardware interfaces", "LiDAR-to-gate perception"],
    },
    {
      x: 8.8,
      title: "Experimental validation",
      accent: C.green,
      img: A.telemetry,
      items: ["Structured navigation", "Unstructured navigation", "Mixed road / gate behavior"],
    },
  ];
  cols.forEach((c) => {
    addImage(slide, c.img, c.x, 1.7, 3.55, 2.18, {
      altText: c.title,
    });
    addRule(slide, c.x, 4.15, 0.75, c.accent, 4);
    addText(slide, c.title, c.x, 4.32, 3.55, 0.38, {
      fontSize: 18,
      bold: true,
      color: C.navy,
    });
    addBulletLines(slide, c.items, c.x + 0.03, 4.83, 3.48, 0.48);
  });
  addText(
    slide,
    "Contribution boundary",
    0.66,
    6.56,
    1.7,
    0.22,
    { fontSize: 9, bold: true, color: C.blue, charSpacing: 0.8 }
  );
  addText(
    slide,
    "The planner was not redesigned; the infrastructure required to execute, observe and validate it on real robots was engineered.",
    2.35,
    6.48,
    10.1,
    0.4,
    {
      fontSize: 15.5,
      bold: true,
      color: C.navy,
    }
  );
  addNotes(
    slide,
    "0:50",
    "Treat the three columns as one stack: simulator, hardware bridge, and evidence.",
    "The contribution is broader than connecting a planner to motors. I developed a custom simulator with synthetic sensing and reporting, the hardware integration including firmware and a structured serial protocol, and the analysis pipeline used across structured, unstructured and mixed navigation. This is the engineering layer that turns an algorithm into an experimentally testable robotic system."
  );
}

// Slide 6 — System demonstration
{
  const slide = pptx.addSlide("RESEARCH");
  addTitle(
    slide,
    "System demonstration",
    "The architecture running across simulation and hardware",
    "One navigation logic, observed through the same interface on different physical platforms"
  );

  addImage(slide, A.gui, 0.65, 1.65, 6.1, 4.52, {
    altText: "Simulator running LiDAR-based gate navigation",
  });
  addText(slide, "SIMULATION", 0.68, 6.31, 2.2, 0.22, {
    fontSize: 9,
    bold: true,
    color: C.blue,
    charSpacing: 1.2,
  });

  addImage(slide, A.car, 7.15, 1.65, 2.57, 2.28, {
    altText: "Robot car used for physical validation",
  });
  addImage(slide, A.tracked, 10.08, 1.65, 2.57, 2.28, {
    altText: "Tracked robot used for portability testing",
  });
  addText(slide, "ROBOT CAR", 7.17, 4.08, 1.5, 0.22, {
    fontSize: 8.5,
    bold: true,
    color: C.blue,
    charSpacing: 1,
  });
  addText(slide, "TRACKED ROBOT", 10.1, 4.08, 2.2, 0.22, {
    fontSize: 8.5,
    bold: true,
    color: C.orange,
    charSpacing: 1,
  });

  addNode(slide, 7.15, 4.67, 5.5, 1.5, "Same navigation architecture", "same planner  ·  same navigation logic  ·  different physical platforms", {
    fill: C.navy,
    line: C.navy,
    titleColor: C.white,
    subtitleColor: "DDEBF5",
    titleSize: 19,
    subtitleSize: 10.5,
  });
  addNotes(
    slide,
    "0:40",
    "Let the images carry the slide: simulator first, then the two physical platforms.",
    "This is the system in operation. The simulator exposes sensing, state, selected references and telemetry. The same navigation architecture is then connected to the robot car and the tracked robot. The physical implementation changes, but the high-level decision logic and experimental workflow remain shared."
  );
}

// Slide 7 — Validation
{
  const slide = pptx.addSlide("RESEARCH");
  addTitle(
    slide,
    "Sim-to-real validation",
    "The same decision architecture was evaluated at three levels",
    "L0 ideal simulation  ·  L1 hardware-informed baseline  ·  L2 physical robot"
  );

  addImage(slide, G.tracking, 0.55, 1.5, 8.35, 5.25, {
    contain: true,
    altText: "Thesis plot comparing p95 trajectory tracking error across validation levels",
  });
  addText(slide, "THESIS DATA · TRACKING ERROR P95 ACROSS L0 / L1 / L2", 0.62, 6.7, 4.55, 0.2, {
    fontSize: 8,
    bold: true,
    color: C.muted,
    charSpacing: 1,
  });

  const kpis = [
    ["3 / 3", "navigation modes completed", C.blue, C.pale],
    ["< 40 ms", "planner compute time p95", C.green, C.greenLight],
    ["Main gap", "physical execution effects", C.orange, C.orangeLight],
  ];
  kpis.forEach(([v, t, color, fill], i) => {
    const x = 9.13;
    const y = 1.63 + i * 1.5;
    slide.addShape(pptx.ShapeType.rect, {
      x,
      y,
      w: 3.55,
      h: 1.23,
      fill: { color: fill },
      line: { color, width: 1.2 },
    });
    addText(slide, v, x + 0.22, y + 0.18, 1.18, 0.38, {
      fontSize: i === 2 ? 17 : 21,
      bold: true,
      color,
    });
    addText(slide, t, x + 1.43, y + 0.17, 1.88, 0.48, {
      fontSize: 10.5,
      bold: true,
      color: C.navy,
    });
    if (i === 2) {
      addText(slide, "slippage · heading dynamics · recovery motion", x + 0.22, y + 0.73, 3.08, 0.26, {
        fontSize: 8.5,
        color: C.orange,
        bold: true,
        align: "center",
      });
    }
  });
  addText(
    slide,
    "L0  ideal simulation     ·     L1  hardware-informed baseline     ·     L2  physical robot",
    9.13,
    6.2,
    3.55,
    0.43,
    {
      fontSize: 9.2,
      bold: true,
      color: C.navy,
      align: "center",
    }
  );
  addText(slide, "Evidence first · summary second", 9.13, 6.67, 3.55, 0.24, {
    fontSize: 10.5,
    bold: true,
    color: C.blue,
    align: "center",
  });
  addNotes(
    slide,
    "1:25",
    "Use the real thesis graph as evidence, then summarize the result through the three KPI blocks.",
    "The comparison was performed at three levels: ideal simulation, a hardware-informed simulated baseline and the physical robot. The plot is taken directly from the thesis analysis and reports the p95 tracking error. All navigation modes completed their tasks and planner computation remained below forty milliseconds at p95. The larger gap was not decision failure, but the physical cost of execution: heading dynamics, recovery motion, slip and actuator behavior."
  );
}

// Slide 8 — Portability
{
  const slide = pptx.addSlide("RESEARCH");
  addTitle(
    slide,
    "Platform portability",
    "From robot car to tracked robot",
    "Portability was tested by changing the locomotion model and hardware interface, not the high-level planner."
  );
  addImage(slide, A.car, 0.65, 1.72, 3.45, 3.0, {
    altText: "Robot car platform",
  });
  addImage(slide, A.tracked, 9.23, 1.72, 3.45, 3.0, {
    altText: "Tracked robot platform",
  });
  addText(slide, "ROBOT CAR", 0.68, 4.88, 2.2, 0.24, {
    fontSize: 9,
    bold: true,
    color: C.blue,
    charSpacing: 1.1,
  });
  addText(slide, "TRACKED ROBOT", 10.45, 4.88, 2.2, 0.24, {
    fontSize: 9,
    bold: true,
    color: C.orange,
    align: "right",
    charSpacing: 1.1,
  });

  addArrow(slide, 4.18, 3.2, 0.45, 0, C.navy, 2.3);
  addArrow(slide, 8.72, 3.2, 0.43, 0, C.navy, 2.3);

  slide.addShape(pptx.ShapeType.roundRect, {
    x: 4.72,
    y: 1.72,
    w: 4.0,
    h: 1.38,
    rectRadius: 0.05,
    fill: { color: C.greenLight },
    line: { color: C.green, width: 1.3 },
  });
  addText(slide, "UNCHANGED", 4.95, 1.91, 1.35, 0.22, {
    fontSize: 9,
    bold: true,
    color: C.green,
    charSpacing: 1.1,
  });
  addText(slide, "Planner   ·   Gate Selection   ·   Validation Pipeline", 4.95, 2.31, 3.52, 0.34, {
    fontSize: 12,
    bold: true,
    color: C.navy,
    align: "center",
  });

  slide.addShape(pptx.ShapeType.roundRect, {
    x: 4.72,
    y: 3.34,
    w: 4.0,
    h: 1.38,
    rectRadius: 0.05,
    fill: { color: C.orangeLight },
    line: { color: C.orange, width: 1.3 },
  });
  addText(slide, "CHANGED", 4.95, 3.53, 1.2, 0.22, {
    fontSize: 9,
    bold: true,
    color: C.orange,
    charSpacing: 1.1,
  });
  addText(slide, "Vehicle Kinematics   ·   Actuation Mapping   ·   Hardware Interface", 4.95, 3.93, 3.52, 0.42, {
    fontSize: 11.5,
    bold: true,
    color: C.navy,
    align: "center",
  });

  addText(
    slide,
    "The framework changed less than the robot.",
    2.0,
    5.75,
    9.33,
    0.48,
    {
      fontSize: 22,
      bold: true,
      color: C.navy,
      align: "center",
    }
  );
  addText(slide, "High-level navigation remained stable; embodiment-specific interfaces were replaced.", 2.15, 6.3, 9.03, 0.32, {
    fontSize: 12,
    color: C.blue,
    bold: true,
    align: "center",
  });
  addNotes(
    slide,
    "1:30",
    "The center block is the thesis in one visual: what stayed fixed and what changed.",
    "The second platform tests the claim that the framework is not specific to the car. The car and tracked robot have different physical behavior, especially during rotation. The execution chain therefore changed in the locomotion model, actuator mapping and hardware interface. The planner, navigation logic, gate selection and validation workflow remained the same. This is the strongest condensed message of the thesis: portability is achieved by isolating embodiment-specific layers."
  );
}

// Slide 9 — 3DOM connection
{
  const slide = pptx.addSlide("RESEARCH");
  addTitle(
    slide,
    "Connection to 3DOM",
    "A shared focus on autonomy that must work in the field",
    "My current framework addresses navigation transfer; 3DOM adds richer perception, mapping and operational environments."
  );

  addText(slide, "MY CURRENT WORK", 0.8, 1.68, 4.4, 0.3, {
    fontSize: 11,
    bold: true,
    color: C.blue,
    charSpacing: 1.2,
  });
  addText(slide, "POTENTIAL 3DOM CONNECTION", 7.25, 1.68, 5.2, 0.3, {
    fontSize: 11,
    bold: true,
    color: C.orange,
    align: "right",
    charSpacing: 1.2,
  });
  const rows = [
    ["LiDAR navigation", "GNSS-denied navigation"],
    ["SLAM foundations", "Collaborative mapping"],
    ["Sim-to-real validation", "Field deployment"],
    ["Mobile robotics", "Autonomous systems"],
    ["State estimation", "Geospatial perception"],
    ["2D mapping", "3D reconstruction"],
  ];
  rows.forEach(([left, right], i) => {
    const y = 2.15 + i * 0.67;
    addNode(slide, 0.78, y, 4.25, 0.48, left, "", {
      fill: i % 2 === 0 ? C.pale : C.white,
      line: C.light,
      titleSize: 12.2,
      align: "left",
    });
    addArrow(slide, 5.32, y + 0.24, 2.65, 0, i % 2 === 0 ? C.cyan : C.blue, 1.5);
    addNode(slide, 8.27, y, 4.25, 0.48, right, "", {
      fill: i % 2 === 0 ? C.orangeLight : C.white,
      line: i % 2 === 0 ? C.orange : C.light,
      titleSize: 12.2,
      align: "right",
    });
  });
  slide.addShape(pptx.ShapeType.roundRect, {
    x: 1.05,
    y: 6.2,
    w: 11.2,
    h: 0.75,
    rectRadius: 0.05,
    fill: { color: C.navy },
    line: { color: C.navy },
  });
  addText(slide, "RESEARCH QUESTION", 1.3, 6.45, 1.75, 0.2, {
    fontSize: 8.5,
    bold: true,
    color: "93D5E4",
    charSpacing: 1,
  });
  addText(
    slide,
    "How can transferable autonomy support field robotics, mapping and spatial perception systems?",
    3.15,
    6.34,
    8.72,
    0.36,
    {
      fontSize: 14,
      bold: true,
      color: C.white,
      align: "center",
    }
  );
  addNotes(
    slide,
    "1:00",
    "Read the slide horizontally: each existing capability opens a richer research problem with 3DOM.",
    "The connection I see is not only thematic. My work contributes a modular navigation and validation base. 3DOM research can extend it toward GNSS-denied operation, richer mapping, geospatial perception and field deployment. The possible collaboration therefore has a clear interface: reuse the transfer and validation framework while introducing more capable sensors, environments and spatial representations."
  );
}

// Slide 10 — Possible research roadmap
{
  const slide = pptx.addSlide("RESEARCH");
  addTitle(
    slide,
    "Possible research roadmap",
    "From thesis prototype to collaborative field autonomy",
    "A platform-neutral progression from controlled validation to field and multi-robot research"
  );

  addText(slide, "COMPLETED", 0.67, 1.68, 2.2, 0.22, {
    fontSize: 8.5,
    bold: true,
    color: C.green,
    charSpacing: 1.2,
  });
  addText(slide, "RESEARCH TRANSITION", 6.42, 1.68, 2.5, 0.22, {
    fontSize: 8.5,
    bold: true,
    color: C.orange,
    charSpacing: 1.2,
  });
  addText(slide, "RESEARCH HORIZON", 9.35, 1.68, 3.2, 0.22, {
    fontSize: 8.5,
    bold: true,
    color: C.blue,
    align: "right",
    charSpacing: 1.2,
  });
  addRule(slide, 0.67, 1.98, 5.35, C.green, 3);
  addRule(slide, 6.4, 1.98, 2.38, C.orange, 3);
  addRule(slide, 9.14, 1.98, 3.44, C.blue, 3);

  addNode(slide, 0.67, 2.32, 1.55, 1.38, "BSc\nThesis", "framework", {
    fill: C.greenLight,
    line: C.green,
    titleSize: 15,
    subtitleSize: 8.5,
  });
  addArrow(slide, 2.28, 3.01, 0.35, 0, C.muted, 1.5);
  addImage(slide, A.car, 2.72, 2.32, 1.83, 1.38, {
    altText: "Robot car",
  });
  addText(slide, "Robot car", 2.73, 3.81, 1.82, 0.22, {
    fontSize: 9,
    bold: true,
    align: "center",
    color: C.navy,
  });
  addArrow(slide, 4.62, 3.01, 0.35, 0, C.muted, 1.5);
  addImage(slide, A.tracked, 5.05, 2.32, 1.83, 1.38, {
    altText: "Tracked robot",
  });
  addText(slide, "Tracked robot", 5.05, 3.81, 1.83, 0.22, {
    fontSize: 9,
    bold: true,
    align: "center",
    color: C.navy,
  });
  addArrow(slide, 6.95, 3.01, 0.35, 0, C.orange, 1.7);
  addNode(slide, 7.4, 2.32, 1.83, 1.38, "Ground\nRobotics", "larger platforms · richer sensing", {
    fill: C.orangeLight,
    line: C.orange,
    titleColor: C.navy,
    titleSize: 15,
    subtitleSize: 8,
  });

  addArrow(slide, 8.31, 3.83, 0, 0.67, C.orange, 1.7);

  const horizon = [
    ["Field\nRobotics", 5.62, C.orangeLight, C.orange],
    ["Spatial\nPerception", 8.02, C.sky, C.blue],
    ["Collaborative\nAutonomy", 10.42, C.greenLight, C.green],
  ];
  horizon.forEach(([title, x, fill, line], i) => {
    addNode(slide, x, 4.62, 2.08, 1.28, title, "", {
      fill,
      line,
      titleSize: 15,
    });
    if (i < horizon.length - 1) addArrow(slide, x + 2.15, 5.26, 0.2, 0, C.blue, 1.5);
  });
  addText(
    slide,
    "Increasing environmental complexity, sensing capability and operational scale",
    5.66,
    6.16,
    6.82,
    0.3,
    {
      fontSize: 11,
      bold: true,
      color: C.blue,
      align: "center",
    }
  );
  addText(
    slide,
    "The thesis is the first experimental layer of a broader research trajectory.",
    1.03,
    6.63,
    11.26,
    0.38,
    {
      fontSize: 17,
      bold: true,
      color: C.navy,
      align: "center",
    }
  );
  addNotes(
    slide,
    "0:40",
    "Move quickly from completed work on the left to the research horizon on the right.",
    "I see the thesis as the first layer of a longer trajectory. The robot car established the complete framework, and the tracked robot tested portability. The next transition is broader ground robotics with richer sensing and less controlled environments. From there, the research can expand toward field robotics, spatial perception and collaborative autonomy without committing the roadmap to one specific platform."
  );
}

// Slide 11 — Possible platforms
{
  const slide = pptx.addSlide("RESEARCH");
  addTitle(
    slide,
    "Possible platforms",
    "Two complementary directions for collaboration",
    "Ground and aerial systems expose different embodiments to the same autonomy and validation questions"
  );
  addPlaceholder(slide, "rover", 0.66, 1.62, 5.75, 2.5, "Leo Rover image", "Ground field robotics");
  addPlaceholder(slide, "uav", 6.92, 1.62, 5.75, 2.5, "UAV image", "Dynamic aerial autonomy");

  addText(slide, "GROUND PLATFORM", 0.68, 4.36, 2.45, 0.25, {
    fontSize: 9,
    bold: true,
    color: C.blue,
    charSpacing: 1.1,
  });
  addText(slide, "AERIAL PLATFORM", 6.94, 4.36, 2.45, 0.25, {
    fontSize: 9,
    bold: true,
    color: C.orange,
    charSpacing: 1.1,
  });
  addBulletLines(slide, ["Outdoor SLAM", "Autonomous exploration", "GNSS-denied field validation"], 0.7, 4.7, 5.35, 0.43);
  addBulletLines(slide, ["Dynamic aerial autonomy", "Mission replanning", "Large-scale monitoring"], 6.96, 4.7, 5.35, 0.43);
  addText(slide, "Example application: wildlife monitoring and anti-poaching support", 6.98, 5.99, 5.23, 0.28, {
    fontSize: 9.5,
    italic: true,
    color: C.muted,
  });

  slide.addShape(pptx.ShapeType.roundRect, {
    x: 0.66,
    y: 6.35,
    w: 11.6,
    h: 0.62,
    rectRadius: 0.06,
    fill: { color: C.navy },
    line: { color: C.navy },
  });
  addText(slide, "SHARED QUESTION", 0.97, 6.53, 1.8, 0.2, {
    fontSize: 9,
    bold: true,
    color: "93D5E4",
    charSpacing: 1.2,
  });
  addText(
    slide,
    "How does the framework transfer across ground and aerial embodiments?",
    2.75,
    6.47,
    8.9,
    0.3,
    {
      fontSize: 15.5,
      bold: true,
      color: C.white,
    }
  );
  addNotes(
    slide,
    "1:00",
    "Give equal time to ground and aerial robotics, then close on the shared scientific question.",
    "Two platform directions could test the framework in complementary ways. A ground platform such as Leo Rover offers continuity through outdoor SLAM, exploration and GNSS-denied validation. A UAV introduces dynamic aerial autonomy, mission replanning and large-scale monitoring; wildlife monitoring and anti-poaching are possible applications rather than the only research objective. The shared question is how the framework transfers across fundamentally different embodiments."
  );
}

// Slide 12 — Long-term interests
{
  const slide = pptx.addSlide("RESEARCH");
  addTitle(
    slide,
    "Long-term interests",
    "Research questions I would like to pursue",
    "A progression from reliable navigation toward collaborative field autonomy"
  );

  const progression = [
    ["Autonomous\nNavigation", C.sky, C.blue],
    ["SLAM &\nMapping", C.greenLight, C.green],
    ["GNSS-denied\nRobotics", C.orangeLight, C.orange],
    ["Field\nRobotics", C.sky, C.blue],
    ["Multi-Robot\nSystems", C.greenLight, C.green],
  ];
  progression.forEach(([title, fill, line], i) => {
    const x = 0.65 + i * 2.52;
    addNode(slide, x, 2.2, 2.08, 1.18, title, "", {
      fill,
      line,
      titleSize: 14.5,
    });
    if (i < progression.length - 1) addArrow(slide, x + 2.15, 2.79, 0.27, 0, C.navy, 1.7);
    addText(slide, `0${i + 1}`, x, 3.52, 2.08, 0.22, {
      fontSize: 8,
      bold: true,
      color: line,
      align: "center",
      charSpacing: 0.9,
    });
  });

  addText(slide, "ENABLING TECHNOLOGIES SUPPORT THE FULL PROGRESSION", 3.18, 4.12, 6.95, 0.22, {
    fontSize: 8.5,
    bold: true,
    color: C.muted,
    align: "center",
    charSpacing: 1.2,
  });
  addRule(slide, 1.46, 4.43, 10.4, C.light, 2);
  addNode(slide, 3.74, 4.7, 2.55, 1.02, "Computer Vision", "semantic and visual cues", {
    fill: C.pale,
    line: C.blue,
    titleSize: 14,
    subtitleSize: 8.5,
  });
  addNode(slide, 7.04, 4.7, 2.55, 1.02, "3D Perception", "geometry and spatial context", {
    fill: C.pale,
    line: C.orange,
    titleSize: 14,
    subtitleSize: 8.5,
  });
  addArrow(slide, 5.02, 4.68, 0, -0.23, C.blue, 1.5);
  addArrow(slide, 8.32, 4.68, 0, -0.23, C.orange, 1.5);
  [1.69, 6.69, 11.69].forEach((x) => addArrow(slide, x, 4.42, 0, -0.66, C.muted, 1.2));

  addText(
    slide,
    "Goal: transferable autonomy that becomes progressively more spatially aware and collaborative.",
    1.34,
    6.37,
    10.65,
    0.36,
    {
      fontSize: 15,
      bold: true,
      color: C.navy,
      align: "center",
    }
  );
  addNotes(
    slide,
    "0:35",
    "Follow the numbered progression from left to right, then identify vision and 3D perception as enabling technologies.",
    "My interests form a progression rather than a collection of independent topics. Reliable navigation leads to SLAM and mapping, then to operation without GNSS, field robotics and eventually multi-robot systems. Computer vision and 3D perception are enabling technologies across this progression. The common goal is autonomy that remains transferable, spatially grounded and experimentally accountable."
  );
}

// Slide 13 — Closing
{
  const slide = pptx.addSlide();
  slide.background = { color: C.white };
  slide.addShape(pptx.ShapeType.rect, {
    x: 0,
    y: 0,
    w: 13.33,
    h: 0.16,
    fill: { color: C.navy },
    line: { color: C.navy },
  });
  addText(slide, "DISCUSSION", 0.7, 0.62, 2.2, 0.24, {
    fontSize: 10,
    bold: true,
    color: C.blue,
    charSpacing: 1.7,
  });
  addText(slide, "Platforms change.\nThe framework keeps evolving.", 0.7, 1.18, 5.1, 1.18, {
    fontSize: 29,
    bold: true,
    color: C.navy,
    valign: "top",
  });
  addText(
    slide,
    "The main outcome of my thesis is not a specific robot, but a modular navigation and validation framework that can be transferred across heterogeneous robotic platforms.",
    0.72,
    2.68,
    4.95,
    1.05,
    {
      fontSize: 15.5,
      color: C.ink,
      bold: true,
      valign: "top",
    }
  );

  slide.addShape(pptx.ShapeType.roundRect, {
    x: 0.72,
    y: 4.18,
    w: 4.95,
    h: 1.15,
    rectRadius: 0.06,
    fill: { color: C.navy },
    line: { color: C.navy },
  });
  addText(slide, "The framework scales across platforms;\nthe research questions evolve with them.", 1.0, 4.43, 4.39, 0.62, {
    fontSize: 17,
    bold: true,
    color: C.white,
    align: "center",
  });

  addText(slide, "PLATFORM EVOLUTION", 7.08, 0.68, 4.55, 0.23, {
    fontSize: 9,
    bold: true,
    color: C.blue,
    align: "center",
    charSpacing: 1.2,
  });
  const chain = [
    { title: "Robot Car", image: A.car, fill: C.sky, line: C.blue },
    { title: "Tracked Robot", image: A.tracked, fill: C.sky, line: C.blue },
    { title: "Ground Robotics", fill: C.orangeLight, line: C.orange },
    { title: "Aerial Robotics", fill: C.pale, line: C.blue },
    { title: "Collaborative Systems", fill: C.greenLight, line: C.green },
  ];
  chain.forEach((item, i) => {
    const y = 1.05 + i * 1.12;
    if (item.image) {
      addImage(slide, item.image, 6.72, y, 1.34, 0.76, {
        altText: item.title,
      });
      addNode(slide, 8.25, y, 3.85, 0.76, item.title, "", {
        fill: item.fill,
        line: item.line,
        titleSize: 14,
      });
    } else {
      addNode(slide, 7.5, y, 4.6, 0.76, item.title, "", {
        fill: item.fill,
        line: item.line,
        titleSize: 14,
      });
    }
    if (i < chain.length - 1) addArrow(slide, 9.8, y + 0.79, 0, 0.27, C.navy, 1.6);
  });

  addText(slide, "Questions & discussion", 0.72, 6.35, 3.2, 0.3, {
    fontSize: 13,
    bold: true,
    color: C.navy,
  });
  addText(slide, "Which transition offers the strongest shared research question?", 0.72, 6.68, 4.95, 0.28, {
    fontSize: 11,
    color: C.blue,
    bold: true,
  });
  addNotes(
    slide,
    "0:30",
    "Close on the visual progression across platforms, then invite discussion.",
    "The platforms can change from the robot car to tracked, ground, aerial and collaborative systems. The durable contribution is the framework that makes this transfer observable and testable. I would be very interested in discussing which of these directions connects most naturally to 3DOM research. Thank you."
  );
}

pptx.writeFile({ fileName: OUT });
