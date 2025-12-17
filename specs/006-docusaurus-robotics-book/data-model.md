# Data Model: Docusaurus Robotics Book

## Overview
This document defines the data model for the Docusaurus-based robotics curriculum, representing the key entities and their relationships that will be implemented in the educational content.

## Core Entities

### Documentation Module
**Description**: Self-contained educational unit covering specific aspects of Physical AI & Humanoid Robotics
**Properties**:
- id: string (unique identifier for the module)
- title: string (display title of the module)
- description: text (brief description of the module content)
- learningObjectives: array of strings (specific learning outcomes for the module)
- prerequisites: array of strings (knowledge/skills required before starting)
- duration: number (estimated time to complete in weeks)
- technologyFocus: array of strings (specific technologies covered in the module)
- difficultyLevel: string (beginner, intermediate, advanced)
- contentPath: string (path to the module's content files)

**Relationships**:
- Contains many WeeklyContent entities
- Contains many LabExercise entities
- Connected to Assessment entities
- Connected to CapstoneProject entity

### Weekly Content
**Description**: Specific learning materials and tasks designated for one week of study within a module
**Properties**:
- id: string (unique identifier for the weekly content)
- moduleId: string (reference to the parent Documentation Module)
- weekNumber: number (sequential week number within the module)
- title: string (title of the weekly content)
- description: text (brief overview of the week's content)
- learningObjectives: array of strings (specific learning outcomes for the week)
- requiredReading: array of strings (list of content files to read)
- practicalExercises: array of strings (list of hands-on tasks to complete)
- estimatedHours: number (time needed to complete the week)

**Relationships**:
- Belongs to one Documentation Module
- Contains many LabExercise entities

### Lab Exercise
**Description**: Hands-on activity requiring specific hardware or software configurations
**Properties**:
- id: string (unique identifier for the lab exercise)
- weekId: string (reference to the parent Weekly Content)
- title: string (title of the lab exercise)
- description: text (overview of what the lab covers)
- objectives: array of strings (what students should learn from the lab)
- requiredEquipment: array of strings (hardware/software needed)
- steps: array of strings (detailed instructions)
- expectedOutcome: text (what success looks like)
- difficultyLevel: string (beginner, intermediate, advanced)

**Relationships**:
- Belongs to one Weekly Content
- Connected to Documentation Module (via Weekly Content)

### Assessment Rubric
**Description**: Guidelines for evaluating student understanding and implementation skills
**Properties**:
- id: string (unique identifier for the assessment rubric)
- moduleId: string (reference to the related Documentation Module)
- title: string (title of the assessment)
- description: text (overview of the assessment)
- evaluationCriteria: array of objects (criteria and point values)
- passingThreshold: number (minimum score required)
- format: string (quiz, project, practical exam, etc.)

**Relationships**:
- Connected to one or more Documentation Modules
- Connected to CapstoneProject if applicable

### Capstone Project
**Description**: Comprehensive final project integrating knowledge from all modules
**Properties**:
- id: string (unique identifier for the capstone project)
- title: string (title of the capstone project)
- description: text (overview of the project)
- learningObjectives: array of strings (what the project demonstrates)
- projectRequirements: array of strings (specific deliverables needed)
- evaluationCriteria: array of objects (grading rubric)
- estimatedDuration: number (time to complete in weeks)
- technologyIntegration: array of strings (technologies to be used)

**Relationships**:
- Connected to all Documentation Modules
- Connected to Assessment Rubric

## Content Structure Model

### Docusaurus Page
**Description**: Individual page within the Docusaurus site
**Properties**:
- id: string (unique identifier for the page)
- path: string (URL path for the page)
- title: string (display title)
- sidebarLabel: string (label to show in sidebar navigation)
- frontmatter: object (YAML metadata at the top of the MD/MDX file)
- content: text (main content of the page)
- isModuleIntro: boolean (whether this is a module introduction page)
- moduleId: string (reference to the related Documentation Module if applicable)

### Navigation Item
**Description**: Element in the site's navigation structure
**Properties**:
- id: string (unique identifier for the navigation item)
- type: string (doc, link, category)
- label: string (display text for the navigation item)
- to: string (URL path if it's a link)
- docId: string (ID of the document if it's a doc link)
- items: array of Navigation Items (sub-items if it's a category)
- position: number (order in the navigation)

## Relationships Summary

- Documentation Module contains multiple Weekly Content items
- Weekly Content contains multiple Lab Exercise items
- Documentation Module can be connected to multiple Assessment Rubrics
- Assessment Rubrics can be connected to multiple Documentation Modules
- Capstone Project connects to all Documentation Modules
- All content items connect to Docusaurus Page model for site rendering
- Navigation Items organize the Docusaurus Page model in the UI

## State Transitions

### Module Progression State
- Planned → In Progress → Reviewing → Completed
- Triggered by student progress and assessment completion

### Content Approval State
- Draft → Review → Approved → Published
- Triggered by curriculum development and review process

## Validation Rules

1. Each Documentation Module must have at least one Weekly Content
2. Each Weekly Content must have at least one Lab Exercise or practical task
3. Learning objectives must be specific, measurable, and achievable
4. Difficulty level must be consistent with prerequisites
5. Estimated duration must be realistic based on content volume
6. Required equipment must be specified for all Lab Exercises
7. Assessment Rubrics must have clear evaluation criteria
8. Capstone Project must integrate knowledge from at least 3 Documentation Modules