# Content Data Model

**Branch**: `001-ros2-humanoid-basics` | **Date**: 2025-12-07

This document defines the conceptual data model for the book's content. As this is a documentation project, the "data" is the content itself.

## Content Entities

### Module

- **Description**: A top-level section of the book, corresponding to a major topic area. It is represented as a directory within `/book/docs`.
- **Attributes**:
    - `title`: The name of the module (e.g., "ROS 2 Basics").
    - `order`: A numeric prefix in the directory name determines its order in the sidebar (e.g., `01-ros2-basics`).
- **Relationships**: Contains one or more **Chapter** entities.

---

### Chapter

- **Description**: A single page or document within a module, focusing on a specific sub-topic. It is represented as a Markdown (`.mdx`) file within a module directory.
- **Attributes**:
    - `title`: The title of the chapter.
    - `order`: A numeric prefix in the file name determines its order within the module (e.g., `01-fundamentals.mdx`).
- **Relationships**:
    - Belongs to one **Module**.
    - May contain zero or more embedded **Diagrams**.
    - May contain zero or more references to **Code Examples**.

---

### Code Example

- **Description**: A runnable piece of source code that demonstrates a concept from a chapter. It is stored as a file (e.g., `.py`) within a corresponding package structure in `/book/src/examples`.
- **Attributes**:
    - `language`: The programming language (e.g., Python).
    - `path`: The file path within the source directory.
- **Relationships**: Referenced by one or more **Chapters**.

---

### Diagram

- **Description**: A visual illustration, such as an architecture diagram or flowchart. It is represented as an image file (`.png`, `.svg`) in `/book/static/img`.
- **Attributes**:
    - `format`: The image file format.
    - `path`: The file path within the static assets directory.
- **Relationships**: Embedded in one or more **Chapters**.
