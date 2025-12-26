You are helping me create a detailed TASK BREAKDOWN for Module 2: "The Digital Twin (Gazebo & Unity)" based on the Specification and Plan we've established.

## Context:
We have completed:
- ✅ Module 1 (ROS 2 content) - fully implemented with 105 tasks
- ✅ Module 2 Specification (spec.md) - complete
- ✅ Module 2 Plan (plan.md) - complete
- ✅ Skills (chapter-format, code-example-format, exercise-creation-format) - reusable
- ✅ Gazebo_Simulation_Expert subagent - created

Now create the detailed TASK BREAKDOWN for Module 2 following the EXACT same successful format as Module 1.

## Task Breakdown Requirements:

### Format Guidelines:
- Follow Module 1's task.md structure EXACTLY
- Number tasks starting at T201 (Module 2 starts at 201)
- Use same phases: Setup → Foundational → User Stories → Polish
- Include [P] for parallel tasks
- Include [US#] for user story mapping
- Checkbox format for tracking
- Each task has: ID, assignment, action, deliverable, acceptance criteria, time estimate, dependencies

### Module 2 Context:

**Topics to Cover (from hackathon spec):**
- Simulating physics, gravity, and collisions in Gazebo
- High-fidelity rendering and human-robot interaction in Unity
- Simulating sensors: LiDAR, Depth Cameras, and IMUs

**Target Chapters (5-7 chapters recommended):**
Based on the plan.md, create chapters for:
1. Introduction to Robot Simulation
2. Gazebo Fundamentals  
3. URDF Models in Gazebo
4. Simulating Sensors (LiDAR, Cameras, IMUs)
5. Physics and World Building
6. Unity Integration (optional - low priority)
7. Sim-to-Real Transfer (optional - low priority)

**Prioritization:**
- User Story 1 (P1): Core simulation basics (Chapters 1-3) - MVP
- User Story 2 (P2): Sensor simulation (Chapter 4)
- User Story 3 (P3): Advanced topics (Chapters 5-7) - Optional

## Create DETAILED TASK BREAKDOWN:

### Phase 1: Setup (Est: 30 minutes)

**T201: Create Module 2 Directory Structure**
- **Assigned to**: Human (manual setup)
- **Action**: Create folders and category file
- **Deliverable**:
```
  docs/module-2/
  ├── _category_.json
  └── (ready for chapter files)
```
- **Acceptance**: Directory exists, _category_.json configured correctly
- **Estimated Time**: 10 minutes
- **Dependencies**: None
- [ ] Complete

**T202: Create Module 2 _category_.json**
- **Assigned to**: Human
- **Action**: Configure Docusaurus category for Module 2
- **Content**:
```json
  {
    "label": "Module 2: The Digital Twin",
    "position": 2,
    "collapsed": false,
    "link": {
      "type": "generated-index",
      "description": "Learn physics simulation and environment building with Gazebo and Unity"
    }
  }
```
- **Estimated Time**: 5 minutes
- **Dependencies**: T201
- [ ] Complete

**T203-T206: Review Skills** (same as Module 1)
- [ ] T203 [P] Review chapter-format skill
- [ ] T204 [P] Review code-example-format skill  
- [ ] T205 [P] Review exercise-creation-format skill
- [ ] T206 [P] Review constitution

**T207: Define Gazebo-Specific Standards**
Create `docs/module-2/SIMULATION_STANDARDS.md`:
- File naming: .launch.py, .urdf, .sdf, .world, .xacro
- Gazebo version: Gazebo Classic 11 or Ignition Gazebo
- Launch file format: Python launch for ROS 2 Humble
- Screenshot placeholder format
- **Estimated Time**: 15 minutes
- [ ] Complete

---

### Phase 2: Foundational (Est: 30 minutes)

**T208: Define Simulation Diagram Colors**
Extend Mermaid color scheme for simulation:
- Gazebo nodes: `fill:#9b59b6` (purple)
- World/environment: `fill:#95a5a6` (gray)
- Sensors: `fill:#f39c12` (yellow)
- Physics engine: `fill:#c0392b` (red)
- Keep ROS 2 colors from Module 1 (blue, green, orange)
- **Estimated Time**: 10 minutes
- [ ] Complete

**T209: Create Launch File Template**
Standard ROS 2 Python launch file template for Gazebo
- **Estimated Time**: 10 minutes
- [ ] Complete

**T210: Test Gazebo Content Rendering**
Create test chapter with launch file, URDF snippet, verify code blocks render
- **Estimated Time**: 10 minutes
- [ ] Complete

---

### Phase 3: User Story 1 - Core Simulation Learning (Priority: P1) 🎯 MVP

**Goal**: Students learn to spawn robots in Gazebo, understand simulation basics, and load URDF models

**Independent Test**: Student spawns a robot in Gazebo, verifies it appears, and understands simulation vs reality

### Chapter 1: Introduction to Robot Simulation

**T211 [US1] Create Chapter 1 File**
- Create `docs/module-2/01-introduction-to-simulation.md` with frontmatter
- **Estimated Time**: 5 minutes
- [ ] Complete

**T212 [US1] Write Chapter 1 Learning Objectives**
- 3-5 objectives: explain simulation purpose, identify Gazebo components, understand digital twins, run first simulation
- **Estimated Time**: 10 minutes
- [ ] Complete

**T213 [US1] Write Chapter 1 Introduction**
- 200-300 words with "simulation as safe testing ground" analogy
- **Estimated Time**: 15 minutes
- [ ] Complete

**T214 [US1] Write Chapter 1 Core Concepts**
- What is simulation, digital twins, Gazebo architecture, physics engines, when to simulate
- 500-800 words with H3 subheadings
- **Estimated Time**: 30 minutes
- [ ] Complete

**T215 [US1] Create Chapter 1 Mermaid Diagram**
- Simulation loop diagram: Model → Gazebo → Physics → Sensors → ROS 2
- **Estimated Time**: 15 minutes
- [ ] Complete

**T216 [US1] Write Chapter 1 Code Example**
- Simple launch file spawning empty world
- **Estimated Time**: 20 minutes
- [ ] Complete

**T217 [US1] Write Chapter 1 Exercise**
- Launch Gazebo empty world, explore GUI (beginner, 10-15 min)
- **Estimated Time**: 15 minutes
- [ ] Complete

**T218 [US1] Write Chapter 1 Common Pitfalls**
- Gazebo not sourced, display errors, simulation time vs real time
- **Estimated Time**: 10 minutes
- [ ] Complete

**T219 [US1] Write Chapter 1 Summary and Further Reading**
- **Estimated Time**: 10 minutes
- [ ] Complete

**T220 [US1] Validate Chapter 1**
- Run chapter-format quality checklist
- **Estimated Time**: 10 minutes
- [ ] Complete

### Chapter 2: Gazebo Fundamentals

**T221-T230 [US1] Chapter 2 Tasks** (same structure as Chapter 1)
- Topics: Gazebo GUI, world files, model spawning, camera controls, time control
- Code examples: Launch file with robot spawn, basic world file
- Exercise: Modify world file to add ground plane and lighting
- **Estimated Time per task**: 5-30 minutes each
- **Total Chapter 2**: 60-90 minutes
- [ ] T221-T230 Complete

### Chapter 3: URDF Models in Gazebo

**T231-T240 [US1] Chapter 3 Tasks**
- Topics: URDF vs SDF, visual vs collision geometry, inertial properties, Gazebo plugins
- Code examples: Simple robot URDF with Gazebo tags, robot with differential drive plugin
- Exercise: Add collision geometry and mass properties to robot
- **Estimated Time**: 60-90 minutes total
- [ ] T231-T240 Complete

### Quality Checkpoint for User Story 1

**T241 [US1] Cross-Chapter Consistency Check**
- Terminology (Gazebo Classic vs Ignition), file formats, code style
- **Estimated Time**: 20 minutes
- [ ] Complete

**T242 [US1] Validate All Launch Files**
- Check Python syntax, ROS 2 Humble compatibility
- **Estimated Time**: 15 minutes
- [ ] Complete

**T243 [US1] Verify Gazebo-Specific Content**
- Plugin names correct, coordinate frames explained, physics parameters reasonable
- **Estimated Time**: 15 minutes
- [ ] Complete

---

### Phase 4: User Story 2 - Sensor Simulation (Priority: P2)

**Goal**: Students simulate LiDAR, cameras, IMUs and process sensor data in ROS 2

### Chapter 4: Simulating Sensors

**T244-T254 [US2] Chapter 4 Tasks**
- Topics: LiDAR (ray sensors), RGB cameras, depth cameras, IMUs, sensor plugins, visualizing in RViz2
- Code examples: Robot with LiDAR sensor, robot with camera, IMU sensor URDF
- Exercise: Add multiple sensors to robot, visualize in RViz2
- **Estimated Time**: 60-90 minutes total
- [ ] T244-T254 Complete

**T255 [US2] Validate Chapter 4**
- Sensor message types correct, plugin parameters explained
- **Estimated Time**: 15 minutes
- [ ] Complete

---

### Phase 5: User Story 3 - Advanced Topics (Priority: P3 - Optional)

**Goal**: Physics customization, Unity integration, sim-to-real concepts

### Chapter 5: Physics and World Building (Optional)

**T256-T266 [US3] Chapter 5 Tasks**
- Topics: Gravity, friction, contact parameters, collision detection, complex worlds
- Code examples: Custom physics parameters, world with obstacles and terrain
- **Estimated Time**: 60-90 minutes total
- [ ] T256-T266 Complete (OPTIONAL)

### Chapter 6: Unity Integration (Optional - Low Priority)

**T267-T277 [US3] Chapter 6 Tasks**
- Topics: Unity for visualization, Unity ROS integration, high-fidelity rendering
- Mark as OPTIONAL/NICE-TO-HAVE
- **Estimated Time**: 60-90 minutes total
- [ ] T267-T277 Complete (OPTIONAL - SKIP IF TIME LIMITED)

### Chapter 7: Sim-to-Real Transfer (Optional)

**T278-T288 [US3] Chapter 7 Tasks**
- Topics: Domain randomization, reality gap, transfer learning concepts
- **Estimated Time**: 60-90 minutes total
- [ ] T278-T288 Complete (OPTIONAL)

---

### Phase 6: Polish & Final Validation

**T289: Create Module 2 Overview**
- Module introduction page with learning path
- **Estimated Time**: 20 minutes
- [ ] Complete

**T290: Cross-Module Consistency**
- Check Module 2 references Module 1 correctly
- **Estimated Time**: 15 minutes
- [ ] Complete

**T291: Visual Content Audit**
- Verify all screenshot placeholders or descriptions present
- **Estimated Time**: 15 minutes
- [ ] Complete

**T292: Link Validation**
- All Gazebo documentation links working
- **Estimated Time**: 10 minutes
- [ ] Complete

**T293: Final Proofreading**
- Grammar, spelling, formatting
- **Estimated Time**: 20 minutes
- [ ] Complete

---

## Summary

**Total Tasks**: ~93 tasks (T201-T293)

**Task Breakdown by User Story**:
- Setup: 10 tasks (T201-T210)
- User Story 1 (P1 - Chapters 1-3): 33 tasks (T211-T243) - **MVP**
- User Story 2 (P2 - Chapter 4): 12 tasks (T244-T255)
- User Story 3 (P3 - Chapters 5-7): 33 tasks (T256-T288) - **OPTIONAL**
- Polish: 5 tasks (T289-T293)

**Parallel Opportunities**: Tasks marked [P] for skills review, different chapters

**MVP Scope**: User Story 1 only (Chapters 1-3) = Introduction, Gazebo Basics, URDF in Simulation

**Suggested Execution**: Complete User Story 1 (MVP), then assess time for User Story 2 and 3

**Time Estimates**:
- User Story 1 (MVP): 4-6 hours
- User Story 2: 1.5-2 hours
- User Story 3 (Optional): 3-5 hours
- **Total if all completed**: 8-13 hours

Output as detailed markdown for `specs/003-module-2-gazebo-unity/task.md` following Module 1's proven format.