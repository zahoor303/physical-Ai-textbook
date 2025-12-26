You are helping me create a detailed SPECIFICATION for Module 1 of the "Physical AI & Humanoid Robotics" textbook.

## Context:
This specification is ONLY for Module 1: "The Robotic Nervous System (ROS 2)". Other modules, chatbot features, and deployment are separate specs.

## Module 1 Scope:
This module teaches students the fundamentals of ROS 2 (Robot Operating System 2), which serves as the communication middleware for controlling robots.

### Topics to Cover:
1. Introduction to ROS 2 and its architecture
2. ROS 2 Nodes - the building blocks
3. Topics - publish/subscribe communication
4. Services - request/response patterns
5. Actions - long-running tasks
6. Understanding URDF (Unified Robot Description Format)
7. Bridging Python AI Agents to ROS 2 controllers using rclpy
8. Practical examples with humanoid robot communication

## Create DETAILED SPECIFICATIONS for Module 1:

### 1. Module Overview
- Module title and tagline
- Learning objectives (5-7 specific outcomes)
- Prerequisites (what students should know before starting)
- Estimated completion time
- Target audience level (beginner/intermediate)

### 2. Chapter Structure
Define exactly how many chapters and what each covers:
- Chapter 1: [Title] - [Brief description]
- Chapter 2: [Title] - [Brief description]
- ... (define all chapters)

For each chapter specify:
- Main concepts to teach
- Estimated word count (e.g., 1000-1500 words)
- Number of code examples needed (e.g., 2-3 examples)
- Diagrams/visuals required (e.g., 1-2 architecture diagrams)

### 3. Chapter Template for Module 1
Every chapter in Module 1 must follow this structure:

**Required Sections:**
- Frontmatter (sidebar_position, title, description)
- Learning Objectives (3-5 bullet points)
- Introduction (150-250 words)
- Core Concepts (500-800 words)
- Code Examples (2-3 working examples)
- Hands-on Exercise (1-2 practical tasks)
- Common Pitfalls (2-3 things to avoid)
- Summary (100-150 words)
- Further Reading (3-5 resources)

### 4. Technical Content Standards for Module 1
- ROS 2 version: Humble (specify this clearly)
- Python version: 3.10+
- Code example requirements:
  - Must be complete and runnable
  - Include installation commands
  - Show expected output
  - Include error handling examples
- Comment density: Every 3-5 lines of code
- Command-line examples format

### 5. Visual Content for Module 1
Specify diagrams needed:
- ROS 2 architecture diagram (Mermaid)
- Node communication flowchart (Mermaid)
- Topic pub/sub visualization (Mermaid)
- URDF structure diagram
- System architecture examples

Format for each:
- Tool to use (Mermaid/image/screenshot)
- Approximate complexity
- Key elements to show

### 6. Code Examples Specification
For each chapter, define:
- Example 1: [What it demonstrates] - Complexity: Beginner
- Example 2: [What it demonstrates] - Complexity: Intermediate
- Example 3: [What it demonstrates] - Complexity: Advanced

**Code Example Template:**
```
1. Problem statement
2. Solution code with comments
3. How to run it
4. Expected output
5. Explanation of key concepts
```

### 7. Exercises and Assessments for Module 1
- Knowledge check questions per chapter (3-5 multiple choice)
- Coding challenges per chapter (1-2 hands-on tasks)
- Module 1 final project specification:
  - Project goal
  - Required functionality
  - Success criteria
  - Estimated time to complete

### 8. Progressive Learning Path
Define how concepts build on each other:
- Chapter 1 teaches X, which is needed for Chapter 2
- Chapter 2 teaches Y, which is needed for Chapter 3
- Identify dependencies between chapters

### 9. File Naming and Organization
```
docs/module-1/
├── _category_.json
├── 01-introduction-to-ros2.md
├── 02-ros2-nodes.md
├── 03-topics-and-publishers.md
├── 04-services.md
├── 05-actions.md
├── 06-urdf-basics.md
└── 07-python-rclpy-integration.md
```

Specify:
- Naming convention (kebab-case, numbered prefixes, etc.)
- Frontmatter requirements
- Category configuration

### 10. Quality Standards for Module 1
- Technical accuracy validation process
- Beginner-friendliness criteria
- Code testing requirements
- Review checklist before marking chapter complete

### 11. External Resources and References
- Official ROS 2 documentation links
- Recommended tutorials
- Video resources (if any)
- Community forums
- GitHub repositories for examples

Think step by step and create a comprehensive specification that focuses ONLY on Module 1 content. Be specific about numbers, structure, and requirements.

Output as a well-organized markdown document with clear sections and subsections.