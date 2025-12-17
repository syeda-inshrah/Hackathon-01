import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Chapter 1: Foundations of Physical AI',
      items: [
        'chapter-1-foundations/index',
        'chapter-1-foundations/week-1',
        'chapter-1-foundations/week-2',
        'chapter-1-foundations/week-3'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: ROS 2 - The Robotic Nervous System',
      items: [
        'chapter-2-ros2/index',
        'chapter-2-ros2/week-1',
        'chapter-2-ros2/week-2',
        'chapter-2-ros2/week-3'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3: Digital Twins with Gazebo & Unity',
      items: [
        'chapter-3-digital-twins/index',
        'chapter-3-digital-twins/week-1',
        'chapter-3-digital-twins/week-2',
        'chapter-3-digital-twins/week-3'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 4: NVIDIA Isaac - The AI Brain',
      items: [
        'chapter-4-nvidia-isaac/index',
        'chapter-4-nvidia-isaac/week-1',
        'chapter-4-nvidia-isaac/week-2',
        'chapter-4-nvidia-isaac/week-3'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 5: Humanoid Locomotion & Manipulation',
      items: [
        'chapter-5-humanoid-locomotion/index',
        'chapter-5-humanoid-locomotion/week-1',
        'chapter-5-humanoid-locomotion/week-2',
        'chapter-5-humanoid-locomotion/week-3'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 6: Vision-Language-Action (VLA)',
      items: [
        'chapter-6-vla/index',
        'chapter-6-vla/week-1',
        'chapter-6-vla/week-2',
        'chapter-6-vla/week-3'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 7: Capstone Project',
      items: [
        'chapter-7-capstone/index',
        'chapter-7-capstone/project-overview',
        'chapter-7-capstone/implementation-guide',
        'chapter-7-capstone/evaluation-criteria'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 8: Hardware & Lab Infrastructure',
      items: [
        'chapter-8-hardware/index',
        'chapter-8-hardware/workstation-requirements',
        'chapter-8-hardware/jetson-kits',
        'chapter-8-hardware/robot-options'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 9: Assessments & Outcomes',
      items: [
        'chapter-9-assessments/index',
        'chapter-9-assessments/quizzes'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 10: Conclusion & Next Steps',
      items: [
        'chapter-10-conclusion/index',
        'chapter-10-conclusion/next-steps'
      ],
    }
  ],
};

export default sidebars;
