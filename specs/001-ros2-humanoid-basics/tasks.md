# Tasks: Module 1: The Robotic Nervous System (ROS 2)

**Input**: Design documents from `specs/001-ros2-humanoid-basics/`
**Prerequisites**: plan.md, spec.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Docusaurus Project Initialization)

**Purpose**: Create the basic Docusaurus site structure and configuration.

- [x] T001 Create the root directory for the Docusaurus project: `book/`
- [x] T002 Initialize a new Docusaurus site using the `classic` template within the `book/` directory.
- [x] T003 Configure the book's title, project details, and sidebar navigation in `book/docusaurus.config.js`.
- [x] T004 Delete the placeholder documentation files created by the Docusaurus initializer in `book/docs/`.

---

## Phase 2: User Story 1 - Understand ROS 2 Core Concepts (Priority: P1) 🎯 MVP

**Goal**: The student can understand the fundamental concepts of ROS 2 (Nodes, Topics, Services) and see them in a diagram.

**Independent Test**: The "ROS 2 Fundamentals" chapter can be read, the diagram is visible, and the content is clear and self-contained.

### Implementation for User Story 1

- [x] T005 [US1] Create the directory for the first module: `book/docs/01-ros2-basics/`
- [x] T006 [US1] Create the first chapter file: `book/docs/01-ros2-basics/01-ros-2-fundamentals.mdx`
- [x] T007 [US1] Write the explanatory text for ROS 2 Nodes, Topics, and Services in `book/docs/01-ros2-basics/01-ros-2-fundamentals.mdx`.
- [ ] T008 [P] [US1] Create a clear architectural diagram illustrating a publisher, subscriber, and service. Save it as `book/static/img/ros2-architecture.png`.
- [ ] T009 [US1] Embed the diagram from `book/static/img/ros2-architecture.png` into `book/docs/01-ros2-basics/01-ros-2-fundamentals.mdx`.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently by running the Docusaurus dev server.

---

## Phase 3: User Story 2 - Run ROS 2 Examples (Priority: P2)

**Goal**: The student can run basic ROS 2 Python examples for publisher, subscriber, and service nodes.

**Independent Test**: The code examples can be built and run successfully, and the "Python Agents with rclpy" chapter can be read.

### Implementation for User Story 2

- [ ] T010 [P] [US2] Create the ROS 2 package structure for the examples: `book/src/examples/ros2_basics/`
- [ ] T011 [P] [US2] Create the ROS 2 package manifest file: `book/src/examples/package.xml`
- [ ] T012 [P] [US2] Create the Python package setup file: `book/src/examples/setup.py`
- [ ] T013 [P] [US2] Implement the simple publisher node in `book/src/examples/ros2_basics/simple_publisher.py`.
- [ ] T014 [P] [US2] Implement the simple subscriber node in `book/src/examples/ros2_basics/simple_subscriber.py`.
- [ ] T015 [P] [US2] Implement the simple service server and client nodes in `book/src/examples/ros2_basics/simple_service.py`.
- [ ] T016 [US2] Create the second chapter file: `book/docs/01-ros2-basics/02-python-agents-with-rclpy.mdx`
- [ ] T017 [US2] Write the explanatory text for the chapter and embed code snippets from the example files into `book/docs/01-ros2-basics/02-python-agents-with-rclpy.mdx`.
- [ ] T018 [US2] Add instructions on how to build the ROS 2 package and run the examples to the chapter.

**Checkpoint**: At this point, User Stories 1 AND 2 should both be functional.

---

## Phase 4: User Story 3 - Control a Simulated Robot (Priority: P3)

**Goal**: The student can run a Python agent to control a simple simulated humanoid robot defined in a URDF file.

**Independent Test**: The URDF can be loaded in RViz2, the control agent runs, and the "URDF for Humanoids" chapter can be read.

### Implementation for User Story 3

- [ ] T019 [P] [US3] Create a simple humanoid URDF file named `book/src/examples/ros2_basics/humanoid.urdf`.
- [ ] T020 [US3] Implement a Python control agent script in `book/src/examples/ros2_basics/control_agent.py` that publishes joint commands.
- [ ] T021 [US3] Create the third chapter file: `book/docs/01-ros2-basics/03-urdf-for-humanoids.mdx`.
- [ ] T022 [US3] Write the explanatory text about URDF, the humanoid model, and the control agent in `book/docs/01-ros2-basics/03-urdf-for-humanoids.mdx`.
- [ ] T023 [US3] Add instructions for launching and visualizing the URDF model in RViz2 and running the control agent script.

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Final review and validation of the entire module.

- [ ] T024 Review all content for technical accuracy, clarity, spelling, and grammar.
- [ ] T025 Validate that all code examples build and run as described in the documentation.
- [ ] T026 Build the full Docusaurus site to ensure there are no broken links or build errors by running `npm run build` in the `book/` directory.
- [ ] T027 Run through the `quickstart.md` guide to ensure it is accurate and complete.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Must be completed first.
- **User Stories (Phases 2-4)**: Depend on Setup completion. They are best completed sequentially (US1 → US2 → US3) as they build upon each other conceptually.
- **Polish (Phase 5)**: Depends on all user stories being complete.

### User Story Dependencies

- **User Story 1**: No dependencies on other stories.
- **User Story 2**: Depends on **User Story 1** for conceptual understanding. The code can be written in parallel, but the documentation makes more sense after US1 is done.
- **User Story 3**: Depends on **User Story 2** as the control agent is an extension of the simple publisher concept.

### Parallel Opportunities

- Within **US1**, T007 (writing text) and T008 (creating diagram) can be done in parallel.
- Within **US2**, tasks T010 through T015 (creating the ROS 2 package files) can all be done in parallel.
- Within **US3**, T019 (creating URDF) and T020 (implementing agent) can be done in parallel.
- Once **Setup** is done, different developers could potentially work on different User Stories, but the logical flow suggests a sequential approach for a single developer.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup.
2. Complete Phase 2: User Story 1.
3. **STOP and VALIDATE**: Run the Docusaurus dev server and review the first chapter. This provides the core concepts as a deliverable.

### Incremental Delivery

1. Complete **MVP** (US1).
2. Add **Phase 3 (User Story 2)**. The deliverable now includes runnable code examples.
3. Add **Phase 4 (User Story 3)**. The deliverable now includes a robotics-specific application.
4. Complete **Phase 5 (Polish)** to finalize the module.
