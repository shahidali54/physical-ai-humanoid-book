# Quickstart: Building the Book Locally

**Branch**: `001-ros2-humanoid-basics` | **Date**: 2025-12-07

This guide provides the steps to build and preview the Docusaurus book on your local machine.

## Prerequisites

- **Node.js**: You must have Node.js (version 18 or newer) and npm installed.
- **ROS 2**: To run the Python code examples, you need a working installation of ROS 2 (Humble recommended).

## Local Development

1.  **Navigate to the Book Directory**:
    Open a terminal and change to the `book` directory, which contains the Docusaurus project.
    ```bash
    cd book
    ```

2.  **Install Dependencies**:
    Install the necessary Node.js packages using npm.
    ```bash
    npm install
    ```

3.  **Start the Development Server**:
    Run the `start` command to launch the Docusaurus local development server. This will open a browser window with a live preview of the book.
    ```bash
    npm start
    ```

The website will be available at `http://localhost:3000`. The server will automatically rebuild and reload the page whenever you make changes to the Markdown or configuration files.

## Building for Production

To create a static build of the website, just like it would be deployed to GitHub Pages, run the `build` command.

```bash
npm run build
```

The output will be placed in the `book/build` directory.
