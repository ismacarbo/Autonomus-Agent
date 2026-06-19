const fs = require("fs");
const path = require("path");
const { execFileSync } = require("child_process");
const JSZip = require("jszip");

const projectRoot = "/home/isma/Desktop/Autonomus-Agent";
const inputPptx =
  "/home/isma/Downloads/From_Sim_to_Real_Navigation_to_Future_Robotics_Research_Directions.pptx";
const outputPptx = path.join(
  projectRoot,
  "From_Sim_to_Real_Navigation_to_Future_Robotics_Research_Directions.pptx",
);
const downloadCopy =
  "/home/isma/Downloads/From_Sim_to_Real_Navigation_to_Future_Robotics_Research_Directions_updated.pptx";
const photoDir = "/home/isma/Desktop/thesis/photos_videos/photos & videos";
const assetDir = path.join(projectRoot, "presentation_assets", "updated_photos");

fs.mkdirSync(assetDir, { recursive: true });

function cropImage(sourceName, outputName, width, height, options = {}) {
  const sourcePath = path.join(photoDir, sourceName);
  const outputPath = path.join(assetDir, outputName);
  const args = [sourcePath, "-auto-orient"];

  if (options.preCrop) {
    args.push("-gravity", options.gravity || "center", "-crop", options.preCrop, "+repage");
  }

  args.push(
    "-resize",
    `${width}x${height}^`,
    "-gravity",
    options.gravity || "center",
    "-extent",
    `${width}x${height}`,
    "-quality",
    "93",
    outputPath,
  );

  execFileSync("convert", args, { stdio: "inherit" });
  return outputPath;
}

function createValidationPanel() {
  const outputPath = path.join(assetDir, "validation_multi_metric.png");
  const svgPath = path.join(assetDir, "validation_multi_metric.svg");
  const levels = [
    { name: "L0 ideal", color: "#45AD45" },
    { name: "L1 baseline", color: "#3D8DC2" },
    { name: "L2 hardware", color: "#E04444" },
  ];
  const charts = [
    {
      title: "Tracking error p95 [m]",
      max: 0.12,
      decimals: 3,
      values: [
        [0.032, 0.032, 0.014],
        [0.001, 0.001, 0.003],
        [0.11, 0.11, 0.107],
      ],
    },
    {
      title: "Path length / reference",
      max: 5.2,
      decimals: 2,
      values: [
        [1.003, 1.001, 1.754],
        [0.991, 1.149, 0.696],
        [0.913, 0.913, 4.546],
      ],
    },
    {
      title: "Mission duration [s]",
      max: 60,
      decimals: 1,
      values: [
        [19.1, 19.05, 46.1],
        [32.2, 43.9, 38.85],
        [10.8, 10.8, 52.61],
      ],
    },
    {
      title: "Planner compute time p95 [ms]",
      max: 45,
      decimals: 1,
      values: [
        [28.45, 32.48, 38.24],
        [17.33, 31.79, 31.02],
        [17.62, 36.35, 31.02],
      ],
    },
  ];

  function esc(value) {
    return String(value)
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;");
  }

  function chartSvg(chart, x, y) {
    const width = 870;
    const height = 385;
    const plotX = x + 78;
    const plotY = y + 64;
    const plotW = 758;
    const plotH = 246;
    const categories = ["Structured", "Unstructured", "Mixed"];
    const groupW = plotW / categories.length;
    const barW = 42;
    const barGap = 7;
    const parts = [
      `<rect x="${x}" y="${y}" width="${width}" height="${height}" rx="10" fill="#FFFFFF" stroke="#DCE5EE" stroke-width="2"/>`,
      `<text x="${x + 24}" y="${y + 38}" class="chart-title">${esc(chart.title)}</text>`,
    ];

    for (let tick = 0; tick <= 4; tick += 1) {
      const value = (chart.max * tick) / 4;
      const py = plotY + plotH - (plotH * tick) / 4;
      const label =
        chart.max < 1
          ? value.toFixed(2)
          : Number.isInteger(value)
            ? String(value)
            : value.toFixed(1);
      parts.push(
        `<line x1="${plotX}" y1="${py}" x2="${plotX + plotW}" y2="${py}" stroke="#E7EDF3" stroke-width="1"/>`,
        `<text x="${plotX - 12}" y="${py + 6}" class="tick" text-anchor="end">${label}</text>`,
      );
    }

    categories.forEach((category, categoryIndex) => {
      const center = plotX + groupW * (categoryIndex + 0.5);
      const groupWidth = levels.length * barW + (levels.length - 1) * barGap;
      const startX = center - groupWidth / 2;

      levels.forEach((level, levelIndex) => {
        const value = chart.values[categoryIndex][levelIndex];
        const barHeight = Math.max(2, (value / chart.max) * plotH);
        const barX = startX + levelIndex * (barW + barGap);
        const barY = plotY + plotH - barHeight;
        parts.push(
          `<rect x="${barX}" y="${barY}" width="${barW}" height="${barHeight}" rx="2" fill="${level.color}"/>`,
        );
        if (levelIndex === 2) {
          parts.push(
            `<text x="${barX + barW / 2}" y="${Math.max(plotY + 18, barY - 7)}" class="hardware-value" text-anchor="middle">${value.toFixed(chart.decimals)}</text>`,
          );
        }
      });

      parts.push(
        `<text x="${center}" y="${plotY + plotH + 31}" class="category" text-anchor="middle">${category}</text>`,
      );
    });

    parts.push(
      `<line x1="${plotX}" y1="${plotY + plotH}" x2="${plotX + plotW}" y2="${plotY + plotH}" stroke="#6B7D90" stroke-width="2"/>`,
    );
    return parts.join("");
  }

  const svg = `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="1800" height="930" viewBox="0 0 1800 930">
  <style>
    text { font-family: Arial, sans-serif; fill: #17233A; }
    .legend { font-size: 23px; font-weight: 700; }
    .chart-title { font-size: 28px; font-weight: 700; }
    .tick { font-size: 18px; fill: #53677A; }
    .category { font-size: 20px; font-weight: 600; fill: #33485F; }
    .hardware-value { font-size: 18px; font-weight: 700; fill: #C73535; }
    .footnote { font-size: 18px; fill: #53677A; }
  </style>
  <rect width="1800" height="930" fill="#FFFFFF"/>
  ${levels
    .map(
      (level, index) =>
        `<rect x="${535 + index * 245}" y="25" width="24" height="24" rx="3" fill="${level.color}"/>` +
        `<text x="${570 + index * 245}" y="45" class="legend">${level.name}</text>`,
    )
    .join("")}
  ${chartSvg(charts[0], 20, 70)}
  ${chartSvg(charts[1], 910, 70)}
  ${chartSvg(charts[2], 20, 475)}
  ${chartSvg(charts[3], 910, 475)}
  <text x="900" y="900" text-anchor="middle" class="footnote">Red labels report L2 hardware values; mixed hardware values aggregate the two thesis runs.</text>
</svg>`;

  fs.writeFileSync(svgPath, svg);
  execFileSync(
    "convert",
    [svgPath, "-background", "white", "-density", "180", "-resize", "1800x930", outputPath],
    { stdio: "inherit" },
  );

  return outputPath;
}

const validationPanel = createValidationPanel();

function renderSvgAsset(name, svg, width, height) {
  const svgPath = path.join(assetDir, `${name}.svg`);
  const outputPath = path.join(assetDir, `${name}.png`);
  fs.writeFileSync(svgPath, svg);
  execFileSync(
    "convert",
    [svgPath, "-background", "none", "-density", "180", "-resize", `${width}x${height}`, outputPath],
    { stdio: "inherit" },
  );
  return outputPath;
}

function createArchitecturePanel() {
  const nodes = [
    { x: 30, width: 245, title: "SENSORS", detail: "LiDAR · IMU · Encoders", fill: "#E8F4FA", stroke: "#28A8C7" },
    { x: 330, width: 205, title: "EKF", detail: "state estimation", fill: "#E8F4FA", stroke: "#28A8C7" },
    { x: 590, width: 225, title: "MOTION PLANNER", detail: "decision core", fill: "#0B2341", stroke: "#0B2341", light: true },
    { x: 870, width: 205, title: "LOCAL TRACKER", detail: "local reference", fill: "#EBF3FE", stroke: "#2763D9" },
    { x: 1130, width: 225, title: "CONTROLLER", detail: "speed · yaw-rate", fill: "#EBF3FE", stroke: "#2763D9" },
    { x: 1410, width: 190, title: "ROBOT", detail: "physical platform", fill: "#FFF1E7", stroke: "#E67B2E" },
  ];
  const arrows = nodes.slice(0, -1).map((node, index) => {
    const next = nodes[index + 1];
    return `<line x1="${node.x + node.width + 15}" y1="235" x2="${next.x - 20}" y2="235" class="flow"/>`;
  });
  const blocks = nodes.map(
    (node) => `
      <rect x="${node.x}" y="145" width="${node.width}" height="180" rx="14"
            fill="${node.fill}" stroke="${node.stroke}" stroke-width="4"/>
      <text x="${node.x + node.width / 2}" y="220" text-anchor="middle"
            class="${node.light ? "title-light" : "title"}">${node.title}</text>
      <text x="${node.x + node.width / 2}" y="264" text-anchor="middle"
            class="${node.light ? "detail-light" : "detail"}">${node.detail}</text>`,
  );

  const svg = `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="1640" height="520" viewBox="0 0 1640 520">
  <defs>
    <marker id="arrowBlue" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto">
      <path d="M0,0 L12,6 L0,12 Z" fill="#1767B0"/>
    </marker>
    <marker id="arrowOrange" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto">
      <path d="M0,0 L12,6 L0,12 Z" fill="#E67B2E"/>
    </marker>
  </defs>
  <style>
    text { font-family: Arial, sans-serif; }
    .title { font-size: 27px; font-weight: 700; fill: #0B2341; }
    .detail { font-size: 21px; fill: #53677A; }
    .title-light { font-size: 27px; font-weight: 700; fill: #FFFFFF; }
    .detail-light { font-size: 21px; fill: #DCE9F7; }
    .label { font-size: 22px; font-weight: 700; }
    .flow { stroke: #1767B0; stroke-width: 10; marker-end: url(#arrowBlue); }
  </style>
  <rect width="1640" height="520" fill="#FFFFFF"/>
  <rect x="12" y="78" width="540" height="280" rx="18" fill="#F1F9FC" stroke="#28A8C7" stroke-width="2" stroke-dasharray="9 7"/>
  <text x="282" y="112" text-anchor="middle" class="label" fill="#1887A4">PLATFORM ADAPTATION · SENSING</text>
  <rect x="575" y="78" width="255" height="280" rx="18" fill="#F3F6FA" stroke="#0B2341" stroke-width="2"/>
  <text x="702" y="112" text-anchor="middle" class="label" fill="#0B2341">PLANNER CORE · UNCHANGED</text>
  <rect x="850" y="78" width="770" height="280" rx="18" fill="#F4F7FC" stroke="#2763D9" stroke-width="2" stroke-dasharray="9 7"/>
  <text x="1235" y="112" text-anchor="middle" class="label" fill="#2763D9">PLATFORM ADAPTATION · CONTROL &amp; ACTUATION</text>
  ${arrows.join("")}
  ${blocks.join("")}
  <path d="M1505 345 C1505 470, 150 470, 150 345" fill="none" stroke="#E67B2E"
        stroke-width="9" stroke-dasharray="18 10" marker-end="url(#arrowOrange)"/>
  <text x="825" y="455" text-anchor="middle" class="label" fill="#C85F18">FEEDBACK · measurements and telemetry</text>
</svg>`;
  return renderSvgAsset("architecture_pipeline", svg, 1640, 520);
}

function createRoadmapArrow() {
  const chevrons = [250, 500, 750, 1000, 1250].map(
    (x) => `<path d="M${x - 18},24 L${x + 4},45 L${x - 18},66" fill="none" stroke="#1E66D0" stroke-width="10" stroke-linecap="round" stroke-linejoin="round"/>`,
  );
  const svg = `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="1500" height="90" viewBox="0 0 1500 90">
  <defs>
    <marker id="arrow" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto">
      <path d="M0,0 L12,6 L0,12 Z" fill="#1E66D0"/>
    </marker>
  </defs>
  <line x1="20" y1="45" x2="1460" y2="45" stroke="#1E66D0" stroke-width="12" marker-end="url(#arrow)"/>
  ${chevrons.join("")}
</svg>`;
  return renderSvgAsset("roadmap_progression_arrow", svg, 1500, 90);
}

function createSequenceArrow() {
  const svg = `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="220" height="90" viewBox="0 0 220 90">
  <path d="M10 45 H170" fill="none" stroke="#1767D9" stroke-width="18" stroke-linecap="round"/>
  <path d="M145 12 L205 45 L145 78 Z" fill="#1767D9"/>
</svg>`;
  return renderSvgAsset("sequence_arrow", svg, 220, 90);
}

const architecturePanel = createArchitecturePanel();
const roadmapArrow = createRoadmapArrow();
const sequenceArrow = createSequenceArrow();

const replacements = [
  {
    slide: 4,
    relId: "rId4",
    media: "custom_slide04_architecture.png",
    file: architecturePanel,
  },
  {
    slide: 1,
    relId: "rId4",
    media: "custom_slide01_hero.jpg",
    file: cropImage("DSC_0045.JPG", "custom_slide01_hero.jpg", 1050, 1200, {
      gravity: "east",
    }),
  },
  {
    slide: 2,
    relId: "rId3",
    media: "custom_slide02_car.jpg",
    file: cropImage("DSC_0039.JPG", "custom_slide02_car.jpg", 1594, 1200),
  },
  {
    slide: 2,
    relId: "rId4",
    media: "custom_slide02_tracked.jpg",
    file: cropImage("DSC_0050.JPG", "custom_slide02_tracked.jpg", 1291, 1200),
  },
  {
    slide: 3,
    relId: "rId4",
    media: "custom_slide03_physical.jpg",
    file: cropImage("DSC_0070.JPG", "custom_slide03_physical.jpg", 1654, 1200),
  },
  {
    slide: 5,
    relId: "rId4",
    media: "custom_slide05_simulation.png",
    file: cropImage(
      "simulation mixed car.png",
      "custom_slide05_simulation.png",
      2040,
      1200,
    ),
  },
  {
    slide: 5,
    relId: "rId5",
    media: "custom_slide05_car.jpg",
    file: cropImage("DSC_0075.JPG", "custom_slide05_car.jpg", 2040, 1200, {
      gravity: "center",
    }),
  },
  {
    slide: 5,
    relId: "rId6",
    media: "custom_slide05_validation.png",
    file: cropImage(
      "diagnostics & plots.png",
      "custom_slide05_validation.png",
      2040,
      1200,
    ),
  },
  {
    slide: 6,
    relId: "rId4",
    media: "custom_slide06_simulation.png",
    file: cropImage(
      "Screenshot From 2026-05-21 03-30-38.png",
      "custom_slide06_simulation.png",
      2180,
      1200,
    ),
  },
  {
    slide: 6,
    relId: "rId5",
    media: "custom_slide06_car.jpg",
    file: cropImage("DSC_0071.JPG", "custom_slide06_car.jpg", 1633, 1200, {
      gravity: "southeast",
    }),
  },
  {
    slide: 6,
    relId: "rId6",
    media: "custom_slide06_tracked.jpg",
    file: cropImage("DSC_0123.JPG", "custom_slide06_tracked.jpg", 1633, 1200),
  },
  {
    slide: 7,
    relId: "rId4",
    media: "custom_slide07_multi_metric.png",
    file: validationPanel,
  },
  {
    slide: 8,
    relId: "rId4",
    media: "custom_slide08_car.jpg",
    file: cropImage("DSC_0040.JPG", "custom_slide08_car.jpg", 1400, 1200),
  },
  {
    slide: 8,
    relId: "rId5",
    media: "custom_slide08_tracked.jpg",
    file: cropImage("DSC_0046.JPG", "custom_slide08_tracked.jpg", 1400, 1200),
  },
  {
    slide: 10,
    relId: "rId4",
    media: "custom_slide10_progression.png",
    file: roadmapArrow,
  },
  {
    slide: 10,
    relId: "rId5",
    media: "custom_slide10_car.jpg",
    file: cropImage("DSC_0074.JPG", "custom_slide10_car.jpg", 1800, 1200),
  },
  {
    slide: 12,
    relId: "rId4",
    media: "custom_slide12_arrow.png",
    file: sequenceArrow,
  },
  {
    slide: 10,
    relId: "rId6",
    media: "custom_slide10_tracked.jpg",
    file: cropImage("DSC_0044.JPG", "custom_slide10_tracked.jpg", 1800, 1200, {
      preCrop: "58%x100%+0+0",
      gravity: "east",
    }),
  },
  {
    slide: 13,
    relId: "rId4",
    media: "custom_slide13_car.jpg",
    file: cropImage("DSC_0037.JPG", "custom_slide13_car.jpg", 2115, 1200),
  },
  {
    slide: 13,
    relId: "rId5",
    media: "custom_slide13_tracked.jpg",
    file: cropImage("DSC_0045.JPG", "custom_slide13_tracked.jpg", 2115, 1200, {
      preCrop: "58%x100%+0+0",
      gravity: "east",
    }),
  },
];

const slideTextReplacements = {
  3: [
    [
      "Research question: how can the same navigation framework be transferred to real, heterogeneous platforms?",
      "Transfer objective: preserve the navigation framework while adapting sensing, control and actuation to heterogeneous platforms.",
    ],
  ],
  9: [
    ["LiDAR navigation", "Navigation &amp; state estimation"],
    ["GNSS-denied navigation", "GNSS-denied autonomy"],
    ["SLAM foundations", "Mapping foundations"],
    ["Collaborative mapping", "Spatial perception &amp; 3D mapping"],
    ["Mobile robotics", "Mobile robot deployment"],
    ["Autonomous systems", "Autonomous field systems"],
    ["RESEARCH QUESTION", "COLLABORATION VALUE"],
    [
      "How can transferable autonomy support field robotics, mapping and spatial perception systems?",
      "Transferable autonomy connects field robotics with mapping, spatial perception and deployable validation.",
    ],
  ],
  10: [
    [
      "A platform-neutral progression from controlled validation to field and multi-robot research",
      "A progression from controlled validation to deployable, collaborative autonomy",
    ],
    ["BSc Thesis", "Validated Core"],
    ["framework", "simulator · car · tracked robot"],
    ["Ground Robotics", "Field Platforms"],
    ["larger platforms · richer sensing", "outdoor deployment · richer perception"],
    ["Spatial Perception", "3D Perception"],
    [
      "Increasing environmental complexity, sensing capability and operational scale",
      "Higher scale · richer sensing · greater autonomy",
    ],
  ],
  11: [
    [
      "Ground and aerial systems expose different embodiments to the same autonomy and validation questions",
      "Ground and aerial systems expose different embodiments to the same autonomy and validation framework",
    ],
    ["Ground field robotics", "Expected outcome: outdoor autonomous navigation"],
    ["Outdoor SLAM", "Visual-LiDAR SLAM"],
    ["SHARED QUESTION", "SHARED VALIDATION GOAL"],
    [
      "How does the framework transfer across ground and aerial embodiments?",
      "Evaluate one autonomy framework across ground and aerial embodiments using comparable metrics.",
    ],
  ],
  12: [
    [
      "Research questions I would like to pursue",
      "Toward collaborative field autonomy",
    ],
  ],
  13: [
    [
      "The framework scales across platforms; the research questions evolve with them.",
      "The framework scales across platforms; each embodiment introduces new sensing and deployment challenges.",
    ],
    ["QUESTIONS &amp; DISCUSSION", "DISCUSSION"],
    [
      "Which transition offers the strongest shared research question?",
      "Which platform is the best next step for extending the framework?",
    ],
  ],
};

const noteTextReplacements = {
  3: [
    [
      "The thesis started from a practical question: what must be added around a planner before it can operate on a real robot?",
      "The thesis started from a practical objective: identify what must be added around a planner before it can operate on a real robot.",
    ],
  ],
  7: [
    [
      "Visual cue: Use the real thesis graph as evidence, then summarize the result through the three KPI blocks.",
      "Visual cue: Read the four thesis metrics together, then summarize the result through the three KPI blocks.",
    ],
    [
      "The plot is taken directly from the thesis analysis and reports the p95 tracking error.",
      "The four plots are taken directly from the thesis analysis and compare tracking error, path efficiency, mission duration and planner computation.",
    ],
  ],
  9: [
    [
      "The connection I see is not only thematic. My work contributes a modular navigation and validation base. 3DOM research can extend it toward GNSS-denied operation, richer mapping, geospatial perception and field deployment.",
      "The connection I see is not only thematic. My work contributes a modular navigation and validation base. The four strongest links to 3DOM are GNSS-denied autonomy, spatial perception and mapping, field deployment, and autonomous systems.",
    ],
  ],
  10: [
    [
      "The next transition is broader ground robotics with richer sensing and less controlled environments.",
      "The next transition is field deployment on larger platforms with richer perception and less controlled environments.",
    ],
  ],
  11: [
    [
      "Visual cue: Give equal time to ground and aerial robotics, then close on the shared scientific question.",
      "Visual cue: Give equal time to ground and aerial robotics, then explain how each direction stresses the framework differently.",
    ],
    [
      "Two platform directions could test the framework in complementary ways. A ground platform such as Leo Rover offers continuity through outdoor SLAM, exploration and GNSS-denied validation. A UAV introduces dynamic aerial autonomy, mission replanning and large-scale monitoring; wildlife monitoring and anti-poaching are possible applications rather than the only research objective. The shared question is how the framework transfers across fundamentally different embodiments.",
      "I see both directions as scientifically interesting because they stress the framework in different ways. Leo Rover is the most direct continuation of the thesis: it preserves ground mobility while introducing outdoor SLAM, exploration and GNSS-denied validation. The UAV direction is particularly interesting because it would require rethinking planning and autonomy on a fundamentally different embodiment, including dynamic mission replanning and large-scale monitoring.",
    ],
  ],
  13: [
    [
      "Visual cue: Close on the visual progression across platforms, then invite discussion.",
      "Visual cue: Close on the platform progression, then use the final question to open the discussion.",
    ],
    [
      "I would be very interested in discussing which of these directions connects most naturally to 3DOM research. Thank you.",
      "I would be very interested in discussing which platform offers the strongest next step for extending and testing the framework. Thank you.",
    ],
  ],
};

function replaceExact(xml, oldText, newText, label) {
  if (!xml.includes(oldText)) {
    throw new Error(`Text not found in ${label}: ${oldText}`);
  }
  return xml.split(oldText).join(newText);
}

function updateRelationship(xml, relId, mediaName, slideNumber) {
  const relationshipRegex = new RegExp(
    `(<Relationship\\s+Id="${relId}"\\s+Type="[^"]*/image"\\s+Target=")[^"]+("/>)`,
  );
  if (!relationshipRegex.test(xml)) {
    throw new Error(`Image relationship ${relId} not found on slide ${slideNumber}`);
  }
  return xml.replace(relationshipRegex, `$1../media/${mediaName}$2`);
}

function removeExistingCrop(xml, relId) {
  const blipRegex = new RegExp(
    `(<a:blip\\s+r:embed="${relId}">[\\s\\S]*?<\\/a:blip>\\s*)<a:srcRect[^>]*/>`,
    "g",
  );
  return xml.replace(blipRegex, '$1<a:srcRect b="0" l="0" r="0" t="0"/>');
}

function removeBlocksAtY(xml, yValues) {
  const values = new Set(yValues.map(String));
  return xml.replace(
    /<p:(sp|cxnSp)>[\s\S]*?<\/p:\1>/g,
    (block) => {
      const match = block.match(/<a:off x="\d+" y="(\d+)"\/>/);
      return match && values.has(match[1]) ? "" : block;
    },
  );
}

function removeBlocksInYRange(xml, minY, maxY) {
  return xml.replace(
    /<p:(sp|pic|cxnSp)>[\s\S]*?<\/p:\1>/g,
    (block) => {
      const match = block.match(/<a:off x="\d+" y="(\d+)"\/>/);
      if (!match) return block;
      const y = Number(match[1]);
      return y >= minY && y <= maxY ? "" : block;
    },
  );
}

function replaceY(xml, oldY, newY) {
  return xml.replace(
    new RegExp(`(<a:off x="\\d+" y=")${oldY}("/>)`, "g"),
    `$1${newY}$2`,
  );
}

function pictureBlock(id, name, relId, x, y, cx, cy) {
  return `<p:pic>
<p:nvPicPr>
<p:cNvPr id="${id}" name="${name}"/>
<p:cNvPicPr><a:picLocks noChangeAspect="1"/></p:cNvPicPr>
<p:nvPr/>
</p:nvPicPr>
<p:blipFill>
<a:blip r:embed="${relId}"><a:alphaModFix/></a:blip>
<a:stretch><a:fillRect/></a:stretch>
</p:blipFill>
<p:spPr>
<a:xfrm><a:off x="${x}" y="${y}"/><a:ext cx="${cx}" cy="${cy}"/></a:xfrm>
<a:prstGeom prst="rect"><a:avLst/></a:prstGeom>
<a:ln><a:noFill/></a:ln>
</p:spPr>
</p:pic>`;
}

function appendToShapeTree(xml, block) {
  return xml.replace("</p:spTree>", `${block}</p:spTree>`);
}

function resizePicturesByRel(xml, relId, positions) {
  let index = 0;
  return xml.replace(/<p:pic>[\s\S]*?<\/p:pic>/g, (block) => {
    if (!block.includes(`r:embed="${relId}"`) || index >= positions.length) {
      return block;
    }
    const position = positions[index++];
    block = block.replace(
      /<a:off x="\d+" y="\d+"\/>/,
      `<a:off x="${position.x}" y="${position.y}"/>`,
    );
    block = block.replace(
      /<a:ext cx="\d+" cy="\d+"\/>/,
      `<a:ext cx="${position.cx}" cy="${position.cy}"/>`,
    );
    return block;
  });
}

function simplifyConnectionSlide(xml) {
  xml = removeBlocksAtY(xml, [
    4416552,
    4526280,
    4636008,
    5029200,
    5138928,
    5248656,
  ]);

  const rowPositions = [
    [2578608, 2788920],
    [2688336, 2898648],
    [2798064, 3008376],
    [3191256, 3611880],
    [3300984, 3721608],
    [3410712, 3831336],
    [3803904, 4434840],
    [3913632, 4544568],
    [4023360, 4654296],
  ];

  for (const [oldY, newY] of rowPositions) {
    xml = replaceY(xml, oldY, newY);
  }
  return xml;
}

function alignRoadmapSlide(xml) {
  const rowPositions = [
    [4191000, 2571750],
    [4286250, 2667000],
    [5572125, 4450000],
  ];

  for (const [oldY, newY] of rowPositions) {
    xml = replaceY(xml, oldY, newY);
  }
  return xml;
}

function simplifyArchitectureSlide(xml) {
  xml = removeBlocksInYRange(xml, 1714500, 5000000);
  return appendToShapeTree(
    xml,
    pictureBlock(
      990,
      "Architecture Pipeline",
      "rId4",
      571500,
      1660000,
      11049000,
      3500000,
    ),
  );
}

function strengthenRoadmapArrow(xml) {
  return resizePicturesByRel(xml, "rId4", [
    { x: 762000, y: 4050000, cx: 10400000, cy: 300000 },
  ]);
}

function strengthenSequenceArrows(xml) {
  xml = removeBlocksAtY(xml, [2427726]);
  xml = resizePicturesByRel(xml, "rId4", [
    { x: 2564000, y: 2475000, cx: 300000, cy: 150000 },
    { x: 4850000, y: 2475000, cx: 300000, cy: 150000 },
    { x: 7136000, y: 2475000, cx: 300000, cy: 150000 },
  ]);
  return appendToShapeTree(
    xml,
    pictureBlock(
      991,
      "Progression Arrow 4",
      "rId4",
      9422000,
      2475000,
      300000,
      150000,
    ),
  );
}

function separateEnablingTechnologyLabel(xml) {
  return replaceY(xml, 3714750, 5334000);
}

async function main() {
  const input = fs.readFileSync(inputPptx);
  const zip = await JSZip.loadAsync(input);

  for (const replacement of replacements) {
    const relPath = `ppt/slides/_rels/slide${replacement.slide}.xml.rels`;
    const slidePath = `ppt/slides/slide${replacement.slide}.xml`;
    let relXml = await zip.file(relPath).async("string");
    let slideXml = await zip.file(slidePath).async("string");

    relXml = updateRelationship(
      relXml,
      replacement.relId,
      replacement.media,
      replacement.slide,
    );
    slideXml = removeExistingCrop(slideXml, replacement.relId);

    zip.file(relPath, relXml);
    zip.file(slidePath, slideXml);
    zip.file(`ppt/media/${replacement.media}`, fs.readFileSync(replacement.file));
  }

  for (const [slideNumber, pairs] of Object.entries(slideTextReplacements)) {
    const slidePath = `ppt/slides/slide${slideNumber}.xml`;
    let slideXml = await zip.file(slidePath).async("string");
    for (const [oldText, newText] of pairs) {
      slideXml = replaceExact(slideXml, oldText, newText, slidePath);
    }
    zip.file(slidePath, slideXml);
  }

  {
    const slidePath = "ppt/slides/slide7.xml";
    let slideXml = await zip.file(slidePath).async("string");
    slideXml = replaceExact(
      slideXml,
      "THESIS DATA • TRACKING ERROR P95 ACROSS L0 / L1 / L2",
      "THESIS DATA • TRACKING · PATH EFFICIENCY · DURATION · COMPUTE TIME",
      slidePath,
    );
    zip.file(slidePath, slideXml);
  }

  {
    const slidePath = "ppt/slides/slide9.xml";
    let slideXml = await zip.file(slidePath).async("string");
    slideXml = simplifyConnectionSlide(slideXml);
    zip.file(slidePath, slideXml);
  }

  {
    const slidePath = "ppt/slides/slide11.xml";
    let slideXml = await zip.file(slidePath).async("string");
    slideXml = slideXml.replace(
      "<a:t>Dynamic aerial autonomy</a:t>",
      "<a:t>Expected outcome: adaptive aerial missions</a:t>",
    );
    slideXml = slideXml.replace(
      "<a:t>Dynamic aerial autonomy</a:t>",
      "<a:t>Dynamic flight autonomy</a:t>",
    );
    zip.file(slidePath, slideXml);
  }

  {
    const slidePath = "ppt/slides/slide4.xml";
    let slideXml = await zip.file(slidePath).async("string");
    slideXml = simplifyArchitectureSlide(slideXml);
    zip.file(slidePath, slideXml);
  }

  {
    const slidePath = "ppt/slides/slide10.xml";
    let slideXml = await zip.file(slidePath).async("string");
    slideXml = alignRoadmapSlide(slideXml);
    slideXml = strengthenRoadmapArrow(slideXml);
    zip.file(slidePath, slideXml);
  }

  {
    const slidePath = "ppt/slides/slide12.xml";
    let slideXml = await zip.file(slidePath).async("string");
    slideXml = strengthenSequenceArrows(slideXml);
    slideXml = separateEnablingTechnologyLabel(slideXml);
    zip.file(slidePath, slideXml);
  }

  for (const [noteNumber, pairs] of Object.entries(noteTextReplacements)) {
    const notePath = `ppt/notesSlides/notesSlide${noteNumber}.xml`;
    let noteXml = await zip.file(notePath).async("string");
    for (const [oldText, newText] of pairs) {
      noteXml = replaceExact(noteXml, oldText, newText, notePath);
    }
    zip.file(notePath, noteXml);
  }

  const result = await zip.generateAsync({
    type: "nodebuffer",
    compression: "DEFLATE",
    compressionOptions: { level: 6 },
  });

  fs.writeFileSync(outputPptx, result);
  fs.writeFileSync(downloadCopy, result);
  console.log(`Updated presentation: ${outputPptx}`);
  console.log(`Download copy: ${downloadCopy}`);
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});
