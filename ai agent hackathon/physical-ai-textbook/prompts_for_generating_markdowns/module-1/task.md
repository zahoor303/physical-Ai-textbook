You are helping me create a detailed TASK BREAKDOWN for Module 1: "The Robotic Nervous System (ROS 2)" based on the Specification and Plan we've established.

## Context:
We have completed:
- ✅ Constitution (core principles)
- ✅ Specification (Module 1 requirements in specs/002-module-1-ros2-content/spec.md)
- ✅ Plan (execution roadmap in specs/002-module-1-ros2-content/plan.md)
- ✅ Subagent (ROS2_Content_Expert in .claude/agents/)
- ✅ Skills (chapter-format, code-example-format, exercise-creation-format in .claude/skills/)

Now we need to break the plan into ACTIONABLE TASKS that can be executed one by one.

## What is a Task Breakdown?
A task breakdown converts high-level plans into specific, executable work items with:
- Clear deliverables
- Assigned subagents
- Required skills
- Estimated time
- Dependencies
- Acceptance criteria

## Create Task Breakdown for Module 1:

The task breakdown should be organized as a **checklist of sequential tasks** that the ROS2_Content_Expert subagent will execute.

---

### Structure Required:

## Task Breakdown: Module 1 - ROS 2 Content

### Phase 1: Setup and Structure (Est: 30 minutes)

**Task 1.1: Create Module 1 Directory Structure**
- **Assigned to**: Human (manual setup)
- **Action**: Create folders and category file
- **Deliverable**:
```
  docs/module-1/
  ├── _category_.json
  └── (ready for chapter files)
```
- **Acceptance**: Directory exists, _category_.json configured correctly
- **Estimated Time**: 10 minutes
- **Dependencies**: None
- [ ] Complete

**Task 1.2: Create Chapter File Templates**
- **Assigned to**: ROS2_Content_Expert
- **Action**: Create 7 empty chapter files with frontmatter
- **Skills Used**: chapter-format (Section 2 - Frontmatter Template)
- **Deliverable**: 7 .md files with proper frontmatter, ready for content
- **Acceptance**: All files have correct sidebar_position, title, description
- **Estimated Time**: 20 minutes
- **Dependencies**: Task 1.1
- [ ] Complete

---

### Phase 2: Core Chapter Content (Est: 6-8 hours)

For each chapter, break down into sub-tasks:

**Task 2.1: Chapter 1 - Introduction to ROS 2**
- **Assigned to**: ROS2_Content_Expert
- **Action**: Write complete Chapter 1 following chapter-format skill
- **Skills Used**: 
  - chapter-format (all sections)
  - code-example-format (for 2-3 examples)
  - exercise-creation-format (for 1-2 exercises)
- **Deliverable**: `docs/module-1/01-introduction-to-ros2.md` (complete)
- **Content Requirements**:
  - Learning Objectives: 3-5 specific outcomes
  - Introduction: 200-300 words (hook, why, what)
  - Core Concepts: 3-5 major topics with H3 headings
  - Code Examples: 2-3 (minimal node, hello world publisher)
  - Exercises: 1-2 (beginner difficulty)
  - Common Pitfalls: 2-3 (sourcing ROS, node names)
  - Summary: 100-150 words
  - Further Reading: 3-5 resources
  - Mermaid Diagrams: 1-2 (ROS 2 architecture, node communication)
- **Acceptance Criteria**:
  - [ ] All 9 required sections present
  - [ ] Word count: 1500-2500 (excluding code)
  - [ ] 2-3 working code examples with comments
  - [ ] 1-2 exercises with success criteria
  - [ ] At least 1 Mermaid diagram
  - [ ] Passes chapter-format quality checklist (Section 7)
- **Estimated Time**: 60-90 minutes
- **Dependencies**: Task 1.2
- [ ] Complete

**Task 2.2: Chapter 2 - ROS 2 Nodes Deep Dive**
[Repeat same structure as 2.1 with chapter-specific content]
- **Estimated Time**: 60-90 minutes
- **Dependencies**: Task 2.1 (concepts build on each other)
- [ ] Complete

**Task 2.3: Chapter 3 - Topics and Publishers**
[Repeat structure]
- **Estimated Time**: 60-90 minutes
- **Dependencies**: Task 2.2
- [ ] Complete

**Task 2.4: Chapter 4 - Subscribers and Callbacks**
[Repeat structure]
- **Estimated Time**: 60-90 minutes
- **Dependencies**: Task 2.3
- [ ] Complete

**Task 2.5: Chapter 5 - Services (Request/Response)**
[Repeat structure]
- **Estimated Time**: 60-90 minutes
- **Dependencies**: Task 2.4
- [ ] Complete

**Task 2.6: Chapter 6 - Actions (Long-Running Tasks)**
[Repeat structure]
- **Estimated Time**: 60-90 minutes
- **Dependencies**: Task 2.5
- [ ] Complete

**Task 2.7: Chapter 7 - URDF and Robot Description**
[Repeat structure]
- **Estimated Time**: 60-90 minutes
- **Dependencies**: Task 2.6
- [ ] Complete

---

### Phase 3: Quality Assurance (Est: 1-2 hours)

**Task 3.1: Cross-Chapter Consistency Check**
- **Assigned to**: ROS2_Content_Expert
- **Action**: Review all 7 chapters for consistency
- **Checks**:
  - [ ] Terminology consistent across chapters
  - [ ] Code style uniform (PEP 8)
  - [ ] Difficulty progression logical
  - [ ] Cross-references work (links to other chapters)
  - [ ] No contradicting information
- **Estimated Time**: 30 minutes
- **Dependencies**: Tasks 2.1-2.7 all complete
- [ ] Complete

**Task 3.2: Code Example Validation**
- **Assigned to**: ROS2_Content_Expert (or Human if testing possible)
- **Action**: Verify all code examples
- **Checks**:
  - [ ] All examples have language identifiers
  - [ ] Comments every 3-5 lines
  - [ ] Expected output shown
  - [ ] Installation commands included
  - [ ] (Optional) Test examples if ROS 2 environment available
- **Estimated Time**: 45 minutes
- **Dependencies**: Task 3.1
- [ ] Complete

**Task 3.3: Exercise Validation**
- **Assigned to**: ROS2_Content_Expert
- **Action**: Verify all exercises are complete and appropriate
- **Checks**:
  - [ ] All exercises have success criteria (3-5 items)
  - [ ] Hints provided (2-4 per exercise)
  - [ ] Extension challenges included
  - [ ] Time estimates reasonable
  - [ ] Difficulty labels accurate
- **Estimated Time**: 30 minutes
- **Dependencies**: Task 3.2
- [ ] Complete

**Task 3.4: Final Proofreading**
- **Assigned to**: ROS2_Content_Expert
- **Action**: Grammar, spelling, formatting check
- **Checks**:
  - [ ] No spelling errors
  - [ ] Proper markdown formatting
  - [ ] Links work (internal and external)
  - [ ] Headings hierarchy correct (H1→H2→H3)
  - [ ] No Lorem Ipsum or placeholder text
- **Estimated Time**: 30 minutes
- **Dependencies**: Task 3.3
- [ ] Complete

---

### Phase 4: Integration and Testing (Est: 30 minutes)

**Task 4.1: Build and Preview in Docusaurus**
- **Assigned to**: Human (requires npm start)
- **Action**: Run Docusaurus dev server and preview Module 1
- **Commands**:
```bash
  npm start
  # Navigate to Module 1 in browser
  # Check all chapters render correctly
```
- **Checks**:
  - [ ] All 7 chapters appear in sidebar
  - [ ] Navigation works between chapters
  - [ ] Code blocks syntax highlight correctly
  - [ ] Mermaid diagrams render
  - [ ] No broken links
- **Estimated Time**: 15 minutes
- **Dependencies**: Task 3.4
- [ ] Complete

**Task 4.2: Git Commit Module 1**
- **Assigned to**: Human
- **Action**: Commit Module 1 to repository
- **Commands**:
```bash
  git add docs/module-1/
  git commit -m "feat: Complete Module 1 - ROS 2 content"
  git push
```
- **Estimated Time**: 5 minutes
- **Dependencies**: Task 4.1
- [ ] Complete

---

## Additional Instructions:

1. **For each chapter task (2.1-2.7)**, provide the SPECIFIC content requirements based on the spec.md:
   - What topics to cover
   - What code examples to include
   - What exercises to create
   - What diagrams needed

2. **Time estimates** should be realistic:
   - First chapter (2.1): 90 minutes (learning curve)
   - Middle chapters (2.2-2.6): 60-75 minutes (faster with practice)
   - Last chapter (2.7): 60 minutes (momentum built)

3. **Dependencies** are critical:
   - Tasks must be done in order when concepts build on each other
   - Some tasks can be parallel (e.g., quality checks)

4. **Acceptance criteria** must be measurable:
   - Use checkboxes for verification
   - Reference specific skill sections
   - Include quantitative measures (word count, number of examples)

5. **Skills referenced** should cite specific sections:
   - "chapter-format (Section 3.4 - Core Concepts)"
   - "code-example-format (Section 2 - Quality Checklist)"

Create a comprehensive, actionable task breakdown that can be executed step-by-step. Each task should be specific enough that the ROS2_Content_Expert subagent knows EXACTLY what to do.

Output as a detailed markdown document with checkboxes that will be saved in `specs/002-module-1-ros2-content/task.md`