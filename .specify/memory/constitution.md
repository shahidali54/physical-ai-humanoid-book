<!--
Sync Impact Report:
Version change: 0.0.0 → 0.1.0
Modified principles:
  - PRINCIPLE_1_NAME → Technical Accuracy
  - PRINCIPLE_2_NAME → Clear Instructional Writing
  - PRINCIPLE_3_NAME → Reliable RAG Answers
  - PRINCIPLE_4_NAME → Docusaurus Compatibility & Content Structure
  - PRINCIPLE_5_NAME → Backend & Embeddings Standardization
  - PRINCIPLE_6_NAME → Deployment & Book Scope
Added sections:
  - Constraints
  - Success Criteria
Removed sections:
  - None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs:
  - None
-->
# AI-generated Docusaurus book + integrated RAG chatbot on "Physical AI & Humanoid Robotics" Constitution

## Core Principles

### Technical Accuracy
All book content and code examples must be technically accurate, validated against official documentation (ROS 2, Gazebo, Unity, Isaac, VLA), and reflect real SDK syntax (rclpy, FastAPI, ChatKit).

### Clear Instructional Writing
The book content must maintain a simple, educational tone suitable for grade 8-10, featuring clear instructions, consistent code, diagrams, and examples throughout.

### Reliable RAG Answers
The integrated RAG chatbot must provide reliable, factual answers strictly sourced from the book's content, preventing hallucinations, and supporting "answer from selected text" functionality.

### Docusaurus Compatibility & Content Structure
Book content must be Docusaurus-compatible Markdown (MDX allowed) and organized into modules that consistently include theory, explanatory diagrams, and functional code examples.

### Backend & Embeddings Standardization
The project's backend must adhere to the specified stack: FastAPI, Neon Postgres, and Qdrant. OpenAI must be used for all embedding generation.

### Deployment & Book Scope
The entire project, including the book and chatbot, must be fully deployable on GitHub Pages. The book's total length must be between 120 and 200 pages.

## Constraints

- Booksize: 120-200 pages
- Modules must include theory + diagrams + working code
- Backend: FastAPI + Neon + Qdrant
- Embeddings: OpenAI
- Fully deployable on GitHub Pages

## Success Criteria

- Book builds and deploys cleanly via Spec-Kit Plus
- RAG chatbot retrieves accurately, with no hallucinations
- All robotics pipelines are clearly explained and demonstrated.

## Governance

This constitution establishes the foundational principles, standards, constraints, and success criteria for the "Physical AI & Humanoid Robotics" project. Any amendments to this document will follow a semantic versioning approach (MAJOR.MINOR.PATCH), with all changes requiring proper documentation and approval. Compliance with these rules will be a mandatory part of all review processes.

**Version**: 0.1.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
