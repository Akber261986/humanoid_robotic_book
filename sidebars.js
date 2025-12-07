// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: Foundations',
      items: [
        'module1/intro_physical_ai',
        'module1/humanoid_fundamentals',
        'module1/ai_paradigms',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      items: [
        'module2/ros2_setup',
        'module2/gazebo_environment',
        'module2/isaac_sim_intro',
        {
          type: 'category',
          label: 'Examples',
          items: [
            'module2/examples/ros2_gazebo_setup',
            'module2/examples/isaac_sim_setup',
          ],
        },
      ],
    },
  ],
};

export default sidebars;