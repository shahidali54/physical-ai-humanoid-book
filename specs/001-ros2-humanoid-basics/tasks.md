# Tasks: Module 1: The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/001-ros2-humanoid-basics/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Paths are based on the `plan.md` structure.

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the Docusaurus book.

- [x] T001 Create the root directory for the book at `book/`
- [x] T002 Copy the contents of `docusaurus-template/` into the `book/` directory to scaffold the Docusaurus project.
- [x] T003 [P] In the new `book/` directory, run `npm install` to install Docusaurus dependencies.
- [x] T004 [P] Create the directory for this module's documentation: `book/docs/01-ros2-basics/`
- [x] T005 [P] Create the directory for this module's code examples: `book/src/examples/01-ros2-basics/`

---

## Phase 2: User Story 1 - Understand ROS 2 Core Concepts (Priority: P1) 🎯 MVP

**Goal**: A student can read the first chapter and understand the fundamental ROS 2 architecture (Nodes, Topics, Services).

**Independent Test**: The student can correctly draw a diagram showing how two nodes communicate over a topic after reading the chapter.

### Implementation for User Story 1

- [x] T006 [US1] Create the first chapter file: `book/docs/01-ros2-basics/01-ros-2-fundamentals.mdx`
- [x] T007 [US1] Write the content for the "ROS 2 Fundamentals" chapter in `book/docs/01-ros2-basics/01-ros-2-fundamentals.mdx`, explaining Nodes, Topics, and Services.
- [x] T008 [P] [US1] Create a diagram illustrating the ROS 2 architecture (nodes, topics, publisher, subscriber) and save it as `book/static/img/ros2_architecture.svg`.
- [x] T009 [US1] Embed the `ros2_architecture.svg` diagram into `book/docs/01-ros2-basics/01-ros-2-fundamentals.mdx`.

**Checkpoint**: At this point, the first chapter should be readable and contain the core concepts and architecture diagram.

---

## Phase 3: User Story 2 - Run ROS 2 Examples (Priority: P2)

**Goal**: The student can run the provided Python code for a publisher, subscriber, and service to see the concepts in action.

**Independent Test**: The student can successfully execute the example scripts and observe the terminal output matching the documentation.

### Implementation for User Story 2

- [x] T010 [US2] Create the second chapter file: `book/docs/01-ros2-basics/02-python-agents-with-rclpy.mdx`.
- [x] T011 [P] [US2] Create the ROS 2 package structure for the examples: `book/src/examples/01-ros2-basics/ros2_basics/`
- [x] T012 [P] [US2] Create an empty `__init__.py` in `book/src/examples/01-ros2-basics/ros2_basics/`
- [x] T013 [P] [US2] Create a `package.xml` file in `book/src/examples/01-ros2-basics/` for the `ros2_basics` package.
- [x] T014 [P] [US2] Create a `setup.py` file in `book/src/examples/01-ros2-basics/` to make the Python package installable.
- [x] T015 [P] [US2] Implement the simple publisher node in `book/src/examples/01-ros2-basics/ros2_basics/simple_publisher.py`.
- [x] T016 [P] [US2] Implement the simple subscriber node in `book/src/examples/01-ros2-basics/ros2_basics/simple_subscriber.py`.
- [x] T017 [P] [US2] Implement the simple service server node in `book/src/examples/01-ros2-basics/ros2_basics/simple_service_server.py`.
- [x] T018 [US2] Write the content for the "Python Agents with rclpy" chapter in `book/docs/01-ros2-basics/02-python-agents-with-rclpy.mdx`, including instructions and code snippets for running the examples.

**Checkpoint**: At this point, the Python examples should be runnable as a ROS 2 package, and the documentation should explain how to use them.

---

## Phase 4: User Story 3 - Control a Simulated Robot (Priority: P3)

**Goal**: The student can run a Python agent that uses `rclpy` to control a simple, simulated humanoid robot described by a URDF file.

**Independent Test**: The student can run the Python agent and observe the humanoid model in RViz2 responding to commands.

### Implementation for User Story 3

- [x] T019 [US3] Create the third chapter file: `book/docs/01-ros2-basics/03-urdf-for-humanoids.mdx`.
- [x] T020 [P] [US3] Create a simple URDF file for a humanoid robot: `book/src/examples/01-ros2-basics/ros2_basics/simple_humanoid.urdf`.
- [x] T021 [P] [US3] Implement a Python agent script that publishes joint commands: `book/src/examples/01-ros2-basics/ros2_basics/humanoid_control_agent.py`.
- [x] T022 [US3] Write the content for the "URDF for Humanoids" chapter in `book/docs/01-ros2-basics/03-urdf-for-humanoids.mdx`, explaining the URDF file and how to launch and control the robot.

**Checkpoint**: The URDF, control agent, and documentation for the final example should be complete.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements and validation.

- [x] T023 Configure the Docusaurus sidebar in `book/docusaurus.config.ts` to include the new chapters from `01-ros2-basics`.
- [x] T024 Add a brief introduction to the `01-ros2-basics` module, potentially in an `_category_.json` file within `book/docs/01-ros2-basics/`.
- [x] T024a [P] Install `raw-loader` dependency for embedding code snippets.
- [x] T025 Run the full Docusaurus build process to ensure there are no errors: `cd book && npm run build`.
- [x] T026 Manually validate that all code examples in `book/src/examples/01-ros2-basics/` run correctly in a ROS 2 Humble environment.

---

## Dependencies & Execution Order

### Phase Dependencies
- **Setup (Phase 1)**: Can start immediately.
- **User Stories (Phases 2-4)**: Depend on Setup completion. They can proceed sequentially or in parallel.
- **Polish (Phase 5)**: Depends on all user stories being complete.

### User Story Dependencies
- **User Story 1 (P1)**: Depends on Phase 1.
- **User Story 2 (P2)**: Depends on Phase 1. No hard dependency on US1, but builds conceptually on it.
- **User Story 3 (P3)**: Depends on Phase 1. No hard dependency on US1/US2, but builds conceptually on them.

### Within Each User Story
- For each story, create the `.md` file first.
- For US2 and US3, create the source code files (`.py`, `.urdf`) before writing the documentation that refers to them.

---

## Implementation Strategy

### Incremental Delivery
1.  Complete Phase 1: Setup → Docusaurus project is initialized.
2.  Complete Phase 2: User Story 1 → First chapter is readable.
3.  Complete Phase 3: User Story 2 → Core ROS 2 examples are available and documented.
4.  Complete Phase 4: User Story 3 → Final humanoid example is available and documented.
5.  Complete Phase 5: Polish → The module is complete and ready for review.
