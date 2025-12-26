import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    "intro",
    "course-setup",
    {
      type: "category",
      label: "Module 1 — ROS 2 Foundations",
      items: [
        "chapters/embodied-ai",
        "chapters/ros2-fundamentals",
        "chapters/rclpy-nodes",
        "chapters/urdf-tf2"
      ]
    },
    {
      type: "category",
      label: "Module 2 — Digital Twin",
      items: ["chapters/gazebo-simulation", "chapters/unity-simulation"]
    },
    {
      type: "category",
      label: "Module 3 — Isaac + Navigation",
      items: ["chapters/isaac-sim-isaac-ros", "chapters/nav2"]
    },
    {
      type: "category",
      label: "Module 4 — VLA",
      items: ["chapters/vla-whisper-llm-planning"]
    },
    {
      type: "category",
      label: "Capstone",
      items: ["chapters/capstone"]
    },
    {
      type: "category",
      label: "Labs",
      items: [
        "labs/README",
        "labs/module-1-ros2-lab",
        "labs/module-2-gazebo-lab",
        "labs/module-3-isaac-lab",
        "labs/module-4-vla-lab",
        "labs/capstone-lab"
      ]
    },
    {
      type: "category",
      label: "Troubleshooting",
      items: [
        "troubleshooting/README",
        "troubleshooting/ros2-install",
        "troubleshooting/gazebo-issues",
        "troubleshooting/isaac-sim-gpu-drivers"
      ]
    },
    {
      type: "category",
      label: "Reference",
      items: [
        "reference/README",
        "reference/ros2-cheatsheet",
        "reference/glossary",
        "reference/build-deploy-verification",
        "reference/authoring-guide",
        "reference/hardware-lab-architectures",
        "reference/on-prem-lab-bom",
        "reference/ether-lab-bom",
        {
          type: "category",
          label: "Templates",
          items: [
            "templates/README",
            "templates/chapter-template",
            "templates/lab-template",
            "templates/quiz-template"
          ]
        }
      ]
    }
  ]
};

export default sidebars;
