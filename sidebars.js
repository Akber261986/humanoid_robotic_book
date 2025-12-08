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
    {
      type: 'category',
      label: 'Module 3: VLA Integration',
      items: [
        'module3/intro_vla',
        'module3/vla_architectures',
        'module3/openvl-isaaclab',
        {
          type: 'category',
          label: 'Examples',
          items: [
            'module3/examples/vla_command_parsing',
            'module3/examples/visual_grounding',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Deployment',
      items: [
        'module4/sim_to_real',
        'module4/deployment_arch',
        'module4/ros2_hardware_deploy',
        {
          type: 'category',
          label: 'Examples',
          items: [
            'module4/examples/deployment_checklist',
          ],
        },
      ],
    },
  ],
};

export default sidebars;