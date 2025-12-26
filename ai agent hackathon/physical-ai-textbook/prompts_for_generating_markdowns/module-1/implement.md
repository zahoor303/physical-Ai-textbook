Use the ROS2_Content_Expert subagent to create Chapter 1: Introduction to ROS 2.

## Task Reference
Execute tasks T011-T021 from specs/002-module-1-ros2-content/task.md

## Required Skills to Follow
- .claude/skills/chapter-format/SKILL.md (complete chapter structure)
- .claude/skills/code-example-format/SKILL.md (code examples)
- .claude/skills/exercise-creation-format/SKILL.md (hands-on exercises)

## Output File
Create: docs/module-1/01-introduction-to-ros2.md

## Chapter Requirements

### Frontmatter
- sidebar_position: 1
- title: "Introduction to ROS 2"
- description: "Learn what ROS 2 is, why it matters, and run your first robot node"

### Content Sections (all required):

1. **Learning Objectives** (3-5 action-verb based objectives):
   - Explain what ROS 2 is and why it's essential for robotics
   - Identify key components of ROS 2 architecture (nodes, topics, middleware)
   - Create and run a simple "Hello World" ROS 2 node
   - Navigate official ROS 2 documentation

2. **Introduction** (200-300 words):
   - Hook: Use "robots need nervous systems" analogy
   - Why it matters: Connect to real robotics applications
   - What you'll learn: Preview key concepts
   - Prerequisites: Basic Python, command-line familiarity

3. **Core Concepts** (500-800 words with H3 subheadings):
   - What is ROS 2? (middleware framework definition)
   - ROS 2 Architecture (3 layers: application, middleware, OS)
   - Nodes as building blocks
   - Topics for communication
   - Middleware (DDS) overview
   - Include analogies before technical definitions

4. **Mermaid Diagrams** (at least 1, use colors from docs/module-1/DIAGRAM_STANDARDS.md):
   - ROS 2 architecture diagram (3 layers)
   - OR node communication flowchart
   - Use color scheme: publishers (blue), subscribers (green), topics (light gray)

5. **Code Example 1: Minimal Hello World Node** (beginner difficulty):
   - 10-20 lines Python using rclpy
   - Complete, runnable code
   - Comments every 3-5 lines
   - Installation commands
   - Expected output shown
   - Explanation (2-4 sentences)
   - Key takeaway (1 sentence)

6. **Exercise 1: Modify Hello World** (beginner difficulty):
   - Modify node to print custom message with timestamp
   - Numbered step-by-step instructions
   - Success criteria (3-5 measurable checkboxes)
   - 2-4 progressive hints (use collapsible details tags)
   - Extension challenge (optional)
   - Time estimate: 10-15 minutes

7. **Common Pitfalls** (2-3 pitfalls):
   - Forgetting to source ROS 2 setup.bash
   - Node name conflicts
   - ModuleNotFoundError for rclpy
   - Each with: what happens, why, how to fix, how to avoid

8. **Summary** (100-150 words):
   - 3-5 key takeaways as bullet points
   - Preview Chapter 2 (ROS 2 Nodes Deep Dive)

9. **Further Reading** (3-5 resources):
   - Official ROS 2 Humble Documentation
   - ROS 2 Design Principles
   - ROS 2 Beginner Tutorials
   - ROS Discourse Forum
   - Each with brief description

## Quality Standards

- Word count: 1500-2500 words (excluding code)
- Target audience: Complete beginners with basic Python knowledge
- Tone: Conversational, welcoming, encouraging
- All code must target ROS 2 Humble
- Python code must be PEP 8 compliant
- Use analogies to explain complex concepts
- Define all technical terms on first use

## Validation Checklist

Before completing, verify:
- [ ] All 9 required sections present
- [ ] Frontmatter correct
- [ ] 1+ Mermaid diagram with correct color scheme
- [ ] 1+ code example with all required subsections
- [ ] 1+ exercise with success criteria and hints
- [ ] 2-3 common pitfalls addressed
- [ ] Summary with 3-5 takeaways
- [ ] 3-5 further reading resources
- [ ] Word count in range
- [ ] Beginner-friendly language throughout

Create the complete chapter now.