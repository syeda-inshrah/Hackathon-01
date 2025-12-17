# Quickstart Guide: Docusaurus Robotics Book

## Overview
This guide will help you quickly set up and start contributing to the Docusaurus-based robotics curriculum titled "Physical AI & Humanoid Robotics: From Digital Intelligence to Embodied Systems".

## Prerequisites

Before starting, ensure you have the following installed:

- Node.js (v18 or higher)
- npm or yarn
- Git
- Python 3.8+ (for ROS 2 examples)
- Access to ROS 2 Humble Hawksbill environment

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to the Documentation Directory

```bash
cd my-book
```

### 3. Install Dependencies

```bash
npm install
```

### 4. Start the Development Server

```bash
npm start
```

This command will start a local development server and open the documentation site in your default browser at `http://localhost:3000`.

## Project Structure

The documentation is organized as follows:

```
my-book/
├── docs/                    # Documentation content
│   ├── intro.md            # Introduction to the curriculum
│   ├── chapter-1-foundations/   # Chapter 1: Foundations of Physical AI
│   ├── chapter-2-ros2/          # Chapter 2: ROS 2 – The Robotic Nervous System
│   ├── chapter-3-digital-twins/ # Chapter 3: Digital Twins with Gazebo & Unity
│   ├── chapter-4-nvidia-isaac/  # Chapter 4: NVIDIA Isaac – The AI Brain
│   ├── chapter-5-humanoid-locomotion/ # Chapter 5: Humanoid Locomotion & Manipulation
│   ├── chapter-6-vla/           # Chapter 6: Vision-Language-Action (VLA)
│   ├── chapter-7-capstone/      # Chapter 7: Capstone Project
│   ├── chapter-8-hardware/      # Chapter 8: Hardware & Lab Infrastructure
│   ├── chapter-9-assessments/   # Chapter 9: Assessments & Outcomes
│   └── chapter-10-conclusion/   # Chapter 10: Conclusion
├── src/                     # Custom components
├── static/                  # Static assets
├── docusaurus.config.ts     # Docusaurus configuration
├── sidebars.ts             # Navigation sidebar configuration
└── package.json            # Project dependencies
```

## Creating New Content

### Adding a New Page

1. Create a new `.md` or `.mdx` file in the appropriate chapter directory
2. Add frontmatter at the top:

```markdown
---
title: Your Page Title
sidebar_label: Sidebar Label
description: Brief description of the page content
keywords: [list, of, relevant, keywords]
---
```

3. Write your content in Markdown format
4. Add the new page to the `sidebars.ts` file to make it appear in the navigation

### Adding a New Chapter

1. Create a new directory in `docs/` following the naming convention (e.g., `chapter-X-topic/`)
2. Create an `index.md` file as the main entry point for the chapter
3. Add other pages as needed for weekly content
4. Update the `sidebars.ts` file to include your new chapter

## Content Guidelines

### Learning Outcomes

Each page should begin with a "Learning Outcomes" section that clearly states what the student will be able to understand or do after completing the content:

```
## Learning Outcomes

- Understand the basic concepts of Physical AI
- Describe the key components of a robotic system
- Implement a simple ROS 2 node in Python
```

### Code Examples

When including code examples, use appropriate syntax highlighting and always include a title:

```python title="Creating a Simple ROS 2 Publisher Node"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Practical Exercises

Include hands-on exercises after theoretical content to reinforce learning:

```
## Hands-On Exercise

Implement a ROS 2 subscriber that listens to the publisher you created above:

1. Create a new Python file called `minimal_subscriber.py`
2. Follow the publisher example to create a subscriber node
3. Subscribe to the 'topic' and log the received messages
4. Run both nodes to see the communication
```

## Building for Production

To build the static site for production deployment:

```bash
npm run build
```

The built site will be available in the `build/` directory and can be served using any static hosting service.

## Running Tests

The site uses Docusaurus' built-in validation through the build process:

```bash
npm run build
```

If there are any issues with your Markdown syntax or site configuration, the build will fail and provide error messages.

## Contributing Guidelines

1. Follow the existing content structure and formatting patterns
2. Ensure all content aligns with the educational objectives
3. Test code examples to verify they work as expected
4. Maintain the accessibility standards required by the curriculum
5. Use inclusive language appropriate for diverse student populations

## Troubleshooting

### Site isn't loading properly

- Verify that all dependencies are installed (`npm install`)
- Check that there are no syntax errors in your Markdown files
- Verify the `sidebars.ts` configuration is correct

### Build errors

- Check the error message for details about what went wrong
- Verify all file paths in your content are correct
- Ensure frontmatter is properly formatted with valid YAML syntax

## Next Steps

1. Review the existing content structure to understand the curriculum flow
2. Choose a section to contribute to or improve
3. Follow the content guidelines to create or update materials
4. Test your changes locally before submitting