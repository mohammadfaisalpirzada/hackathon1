import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

// ---- Stable defaults (prevents baseUrl flipping locally) ----
const DEFAULT_OWNER = "mohammadfaisalpirzada";
const DEFAULT_REPO = "hackathon1";

// Prefer CI-provided repo, else fall back to real repo name
const repoName = process.env.GITHUB_REPOSITORY?.split("/")[1] ?? DEFAULT_REPO;

// Assumption: deploying to GitHub Pages at https://<owner>.github.io/<repo>/
const owner = process.env.GITHUB_REPOSITORY_OWNER ?? DEFAULT_OWNER;

// Allow overrides via env, otherwise use stable defaults
const url = process.env.DOCUSAURUS_URL ?? `https://${owner}.github.io`;
const baseUrl = process.env.DOCUSAURUS_BASE_URL ?? `/${repoName}/`;

const config: Config = {
  title: "Physical AI & Humanoid Robotics (Quarter 2)",
  tagline: "ROS 2 + simulation for intermediate developers",
  favicon: "img/favicon.svg",

  url,
  baseUrl,

  organizationName: owner,
  projectName: repoName,

  onBrokenLinks: "throw",
  onBrokenMarkdownLinks: "throw",

  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  markdown: {
    mermaid: true,
  },
  themes: ["@docusaurus/theme-mermaid"],

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          routeBasePath: "/",
          editUrl: process.env.GITHUB_REPOSITORY
            ? `https://github.com/${process.env.GITHUB_REPOSITORY}/edit/main/`
            : `https://github.com/${DEFAULT_OWNER}/${DEFAULT_REPO}/edit/main/`,
        },
        blog: false,
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    navbar: {
      title: "Physical AI & Humanoid Robotics (Q2)",
      items: [
        { to: "/", label: "Docs", position: "left" },
        { to: "/labs", label: "Labs", position: "left" },
        { to: "/troubleshooting", label: "Troubleshooting", position: "left" },
        { to: "/reference", label: "Reference", position: "left" },
        {
          href: process.env.GITHUB_REPOSITORY
            ? `https://github.com/${process.env.GITHUB_REPOSITORY}`
            : `https://github.com/${DEFAULT_OWNER}/${DEFAULT_REPO}`,
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Course",
          items: [
            { label: "Intro", to: "/" },
            { label: "Labs", to: "/labs" },
            { label: "Troubleshooting", to: "/troubleshooting" },
            { label: "Reference", to: "/reference" },
            { label: "Capstone", to: "/capstone" },
          ],
        },
      ],
      copyright: `© ${new Date().getFullYear()}`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
