# Physical AI & Humanoid Robotics Book

This repository contains an educational book on Physical AI and Humanoid Robotics, focusing on practical applications of AI in physical systems with emphasis on embodied intelligence.

## Overview

This educational resource provides a comprehensive introduction to Physical AI and Humanoid Robotics, covering:

- **Module 1**: Foundations of Physical AI and Humanoid Robotics
  - Introduction to Physical AI and embodied intelligence
  - Humanoid Robotics fundamentals and concepts
  - AI paradigms in robotics applications

- **Module 2**: Simulation Environments for Humanoid Robotics
  - ROS 2 setup and configuration for robotics
  - Gazebo simulation environment usage
  - NVIDIA Isaac Sim integration and best practices

- **Module 3**: Vision-Language Assistant (VLA) Integration
  - VLA concepts in robotics applications
  - Integration architectures and patterns
  - OpenVLA and Isaac Lab overview

- **Module 4**: Deployment and Real-World Applications
  - Transition from simulation to real-world deployment
  - Architecture considerations for deployment
  - ROS 2 deployment on physical hardware

## Structure

- `docs/` - Contains all book content organized by modules
  - `module1/` - Foundations of Physical AI and Humanoid Robotics
  - `module2/` - Simulation Environments for Humanoid Robotics
  - `module3/` - Vision-Language Assistant Integration (Coming Soon)
  - `module4/` - Deployment and Real-World Applications (Coming Soon)
  - `examples/` - Reproducible code examples for each module

- `docusaurus.config.js` - Docusaurus configuration for the book
- `sidebars.js` - Navigation structure for the book
- `SUMMARY.md` - Executive summary of the entire book

## Quick Start

For a quick introduction to the concepts covered in this book, see the `quickstart.md` file in the specs directory which provides step-by-step setup instructions and foundational examples.

## Target Audience

University students and early-career engineers with computer science/engineering background and introductory AI knowledge. The content balances theoretical foundations with practical implementation.

## Technical Requirements

- **Operating System**: Ubuntu 20.04+ (recommended)
- **Hardware**: NVIDIA GPU with Compute Capability 6.0+ (for Isaac Sim), 16GB+ RAM
- **Software**: Python 3.11, ROS 2 Humble Hawksbill, Gazebo Garden, Isaac Sim
- **Development Environment**: Linux command line proficiency required

## Learning Outcomes

Upon completing this book, readers will be able to:
1. Understand the principles of Physical AI and embodied intelligence
2. Set up and configure simulation environments (ROS 2, Gazebo, Isaac Sim)
3. Implement basic robotic control in simulation environments
4. Appreciate the challenges and opportunities in humanoid robotics
5. Prepare for advanced topics in VLA integration and real-world deployment

## Physical AI & Humanoid Robotics Book

This is an educational book on Embodied Intelligence and Robotics built with Docusaurus.

## Overview

This educational resource provides a comprehensive introduction to Physical AI and Humanoid Robotics, covering:

- **Module 1**: Foundations of Physical AI and Humanoid Robotics
  - Introduction to Physical AI and embodied intelligence
  - Humanoid Robotics fundamentals and concepts
  - AI paradigms in robotics applications

- **Module 2**: Simulation Environments for Humanoid Robotics
  - ROS 2 setup and configuration for robotics
  - Gazebo simulation environment usage
  - NVIDIA Isaac Sim integration and best practices

- **Module 3**: Vision-Language Assistant (VLA) Integration
  - VLA concepts in robotics applications
  - Integration architectures and patterns
  - OpenVLA and Isaac Lab overview

- **Module 4**: Deployment and Real-World Applications
  - Transition from simulation to real-world deployment
  - Architecture considerations for deployment
  - ROS 2 deployment on physical hardware

## Structure

- `docs/` - Contains all book content organized by modules
  - `module1/` - Foundations of Physical AI and Humanoid Robotics
  - `module2/` - Simulation Environments for Humanoid Robotics
  - `module3/` - Vision-Language Assistant Integration (Coming Soon)
  - `module4/` - Deployment and Real-World Applications (Coming Soon)
  - `examples/` - Reproducible code examples for each module

- `docusaurus.config.ts` - Docusaurus configuration for the book
- `sidebars.js` - Navigation structure for the book
- `SUMMARY.md` - Executive summary of the entire book

## Quick Start

For a quick introduction to the concepts covered in this book, see the `quickstart.md` file in the specs directory which provides step-by-step setup instructions and foundational examples.

## Target Audience

University students and early-career engineers with computer science/engineering background and introductory AI knowledge. The content balances theoretical foundations with practical implementation.

## Technical Requirements

- **Operating System**: Ubuntu 20.04+ (recommended)
- **Hardware**: NVIDIA GPU with Compute Capability 6.0+ (for Isaac Sim), 16GB+ RAM
- **Software**: Python 3.11, ROS 2 Humble Hawksbill, Gazebo Garden, Isaac Sim
- **Development Environment**: Linux command line proficiency required

## Learning Outcomes

Upon completing this book, readers will be able to:
1. Understand the principles of Physical AI and embodied intelligence
2. Set up and configure simulation environments (ROS 2, Gazebo, Isaac Sim)
3. Implement basic robotic control in simulation environments
4. Appreciate the challenges and opportunities in humanoid robotics
5. Prepare for advanced topics in VLA integration and real-world deployment

## Deployment

This book is built with [Docusaurus](https://docusaurus.io/) and is configured for GitHub Pages deployment.

### Local Development

1. Install dependencies:
   ```bash
   npm install
   ```

2. Start the development server:
   ```bash
   npm start
   ```

3. Build for production:
   ```bash
   npm run build
   ```

### GitHub Pages Deployment

The site is configured to deploy automatically via GitHub Actions when changes are pushed to the main branch. The GitHub Actions workflow is defined in `.github/workflows/deploy.yml`.

To manually deploy, you can run:
```bash
GIT_USER=Akber261986 CURRENT_BRANCH=main npm run deploy
```

The site will be deployed to: https://Akber261986.github.io/humanoid_robotic_book/