#!/usr/bin/env bash
# Install the GitHub CLI (gh) from GitHub's official apt repository.
#
# Ubuntu 22.04 ships gh 2.4.0 (2021), which is old enough that the auth
# device-flow can misbehave. This pulls a current build instead.
#
# Run with sudo:  sudo ~/rebot-setup/install-gh.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "!! Run with sudo: sudo $0"
  exit 1
fi

ARCH="$(dpkg --print-architecture)"          # arm64 on the Jetson
KEYRING=/usr/share/keyrings/githubcli-archive-keyring.gpg

echo "==> Adding GitHub CLI apt repository ($ARCH)"
apt-get update -qq
apt-get install -y curl gnupg

curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
  -o "$KEYRING"
chmod go+r "$KEYRING"

echo "deb [arch=$ARCH signed-by=$KEYRING] https://cli.github.com/packages stable main" \
  > /etc/apt/sources.list.d/github-cli.list

apt-get update -qq

echo "==> Installing gh"
apt-get install -y gh

echo
echo "=== DONE ==="
gh --version | head -1
cat <<'EOF'

NEXT (run as your normal user, NOT with sudo -- auth is per-user):

  gh auth login
      choose: GitHub.com -> HTTPS -> authenticate with a browser
      (or paste a Personal Access Token with 'repo' scope)

Then publish the private repo:

  cd ~/rebot-arm-private
  gh repo create rebot-arm-private --private --source=. --push

  ^ --private is what keeps it out of public view. Double-check the repo
    shows "Private" on GitHub afterwards.
EOF
