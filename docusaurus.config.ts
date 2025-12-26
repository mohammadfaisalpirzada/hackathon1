import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

const repoName =
  process.env.GITHUB_REPOSITORY?.split("/")[1] ?? "physical-ai-humanoid-robotics-q2";
const isProd = process.env.NODE_ENV === "production";

// Assumption: deploying to GitHub Pages at https://<owner>.github.io/<repo>/
const url =
  process.env.DOCUSAURUS_URL ??
  (process.env.GITHUB_REPOSITORY_OWNER
    ? `https://${process.env.GITHUB_REPOSITORY_OWNER}.github.io`
    : "https://example.com");
const baseUrl = process.env.DOCUSAURUS_BASE_URL ?? (isProd ? `/${repoName}/` : "/");

const config: Config = {
  title: "Physical AI & Humanoid Robotics (Quarter 2)",
  tagline: "ROS 2 + simulation for intermediate developers",
  favicon: "img/favicon.svg",

  url,
  baseUrl,

  organizationName: process.env.GITHUB_REPOSITORY_OWNER ?? "your-org",
  projectName: repoName,

  onBrokenLinks: "throw",

  i18n: {
    defaultLocale: "en",
    locales: ["en"]
  },

  markdown: {
    mermaid: true,
    hooks: {
      onBrokenMarkdownLinks: "throw"
    }
  },
  themes: ["@docusaurus/theme-mermaid"],

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          routeBasePath: "/",
          editUrl:
            process.env.GITHUB_REPOSITORY_OWNER && process.env.GITHUB_REPOSITORY
              ? `https://github.com/${process.env.GITHUB_REPOSITORY}/edit/main/`
              : undefined
        },
        blog: false,
        theme: {
          customCss: "./src/css/custom.css"
        }
      } satisfies Preset.Options
    ]
  ],

  themeConfig: {
    navbar: {
      title: "Physical AI & Humanoid Robotics (Q2)",
      items: [
        { to: "/", label: "Docs", position: "left" },
        { to: "/labs", label: "Labs", position: "left" },
        { to: "/troubleshooting", label: "Troubleshooting", position: "left" },
        { to: "/reference", label: "Reference", position: "left" },
        process.env.GITHUB_REPOSITORY
          ? {
              href: `https://github.com/${process.env.GITHUB_REPOSITORY}`,
              label: "GitHub",
              position: "right"
            }
          : { href: "https://github.com", label: "GitHub", position: "right" }
      ]
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
            { label: "Capstone", to: "/capstone" }
          ]
        }
      ],
      copyright: `© ${new Date().getFullYear()}`
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula
    }
  } satisfies Preset.ThemeConfig
};

export default config;
