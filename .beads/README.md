# Beads - AI-Native Issue Tracking

Welcome to Beads! This repository uses **Beads** for issue tracking - a modern, AI-native tool designed to live directly in your codebase alongside your code.

## What is Beads?

Beads is issue tracking that lives in your repo, making it perfect for AI coding agents and developers who want their issues close to their code. No web UI required - everything works through the CLI and integrates seamlessly with git.

**Learn more:** [github.com/steveyegge/beads](https://github.com/steveyegge/beads)

## Setup / Configuration

### Git Merge Driver

The `.beads/issues.jsonl` file uses a custom git merge driver named `beads` to
intelligently handle 3-way merges during `git pull` or `git merge` operations.
This driver is referenced in `.gitattributes`:

```gitattributes
.beads/issues.jsonl merge=beads
```

**Contributors must configure this merge driver** in their local git config
(or the repo's `.git/config`) so that git will invoke `bd merge` for conflict
resolution.

First, ensure you have the `bd` tool installed:

```bash
# Install beads CLI
curl -sSL https://raw.githubusercontent.com/steveyegge/beads/main/scripts/install.sh | bash
```

Then add the merge driver to your local git config:

```bash
# Configure the beads merge driver
git config merge.beads.name "Beads JSONL 3-way merge"
git config merge.beads.driver "bd merge %O %A %B %P"
```

Or add the following to your `.git/config`:

```ini
[merge "beads"]
    name = Beads JSONL 3-way merge
    driver = bd merge %O %A %B %P
```

### Running Merges Manually

If you need to run the merge tool directly (e.g., for debugging or manual
reconciliation), invoke:

```bash
bd merge <output> <base> <left> <right>
```

Where:

- `<output>` — Path for the merged result
- `<base>` — Common ancestor version
- `<left>` — "Ours" version (current branch)
- `<right>` — "Theirs" version (incoming branch)

Exit codes:

- `0` — Merge successful (no conflicts)
- `1` — Merge completed with conflicts (conflict markers in output)
- `2` — Error (invalid arguments, file not found, etc.)

### Automatic Setup

The easiest way to configure the merge driver is to use the project's install script:

```bash
# Install git hooks AND configure the beads merge driver
./scripts/install-hooks.sh
```

Alternatively, `bd init` can also configure the merge driver:

```bash
# Full initialization (includes merge driver setup)
bd init

# Skip merge driver setup if configuring manually
bd init --skip-merge-driver
```

For contributors joining an existing beads-enabled repo, run:

```bash
bd init --contributor
```

This wizard walks through the necessary local configuration.

---

## Quick Start

### Essential Commands

```bash
# Create new issues
bd create "Add user authentication"

# View all issues
bd list

# View issue details
bd show <issue-id>

# Update issue status
bd update <issue-id> --status in_progress
bd update <issue-id> --status done

# Sync with git remote
bd sync
```

### Working with Issues

Issues in Beads are:

- **Git-native**: Stored in `.beads/issues.jsonl` and synced like code
- **AI-friendly**: CLI-first design works perfectly with AI coding agents
- **Branch-aware**: Issues can follow your branch workflow
- **Always in sync**: Auto-syncs with your commits

## Why Beads?

✨ **AI-Native Design**

- Built specifically for AI-assisted development workflows
- CLI-first interface works seamlessly with AI coding agents
- No context switching to web UIs

🚀 **Developer Focused**

- Issues live in your repo, right next to your code
- Works offline, syncs when you push
- Fast, lightweight, and stays out of your way

🔧 **Git Integration**

- Automatic sync with git commits
- Branch-aware issue tracking
- Intelligent JSONL merge resolution

## Get Started with Beads

Try Beads in your own projects:

```bash
# Install Beads
curl -sSL https://raw.githubusercontent.com/steveyegge/beads/main/scripts/install.sh | bash

# Initialize in your repo
bd init

# Create your first issue
bd create "Try out Beads"
```

## Learn More

- **Documentation**: [github.com/steveyegge/beads/docs](https://github.com/steveyegge/beads/tree/main/docs)
- **Quick Start Guide**: Run `bd quickstart`
- **Examples**: [github.com/steveyegge/beads/examples](https://github.com/steveyegge/beads/tree/main/examples)

---

*Beads: Issue tracking that moves at the speed of thought* ⚡
