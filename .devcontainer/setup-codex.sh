#!/usr/bin/env bash
set -eu

mkdir -p "$HOME/.codex" /workspaces/BoatMotorBoard/.codex

cat > "$HOME/.codex/config.toml" <<'EOF'
model = "gpt-5.5"
model_reasoning_effort = "high"

[projects."/workspaces/BoatMotorBoard"]
trust_level = "trusted"
EOF

cat > /workspaces/BoatMotorBoard/.codex/config.toml <<'EOF'
sandbox_mode = "danger-full-access"
approval_policy = "on-request"
EOF
