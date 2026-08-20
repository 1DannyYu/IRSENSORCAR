#!/bin/bash

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
TMP_ROOT="${TMPDIR:-/tmp}"
STATE_ID="$(printf '%s' "$PROJECT_DIR" | cksum | awk '{print $1}')"
STATE_PREFIX="${TMP_ROOT%/}/car-and-robotic-arm-${STATE_ID}"
LOG_FILE="${STATE_PREFIX}.log"
PID_FILE="${STATE_PREFIX}.pid"
PORT_FILE="${STATE_PREFIX}.port"
BASE_PORT=18427
MAX_PORT=18526
URL=""

homepage_ready() {
  [ -n "$URL" ] && curl -fsS "$URL" >/dev/null 2>&1
}

saved_server_ready() {
  [ -s "$PORT_FILE" ] || return 1

  SAVED_PORT="$(sed -n '1p' "$PORT_FILE")"
  case "$SAVED_PORT" in
    ''|*[!0-9]*) return 1 ;;
  esac

  [ "$SAVED_PORT" -ge "$BASE_PORT" ] || return 1
  [ "$SAVED_PORT" -le "$MAX_PORT" ] || return 1

  SAVED_PID="$(lsof -t -nP -iTCP:"$SAVED_PORT" -sTCP:LISTEN 2>/dev/null | sed -n '1p')"
  [ -n "$SAVED_PID" ] || return 1
  SAVED_CWD="$(lsof -a -p "$SAVED_PID" -d cwd -Fn 2>/dev/null | sed -n 's/^n//p')"
  [ "$SAVED_CWD" = "$PROJECT_DIR" ] || return 1

  URL="http://127.0.0.1:${SAVED_PORT}/IRSENSORCAR/"
  homepage_ready || return 1
  printf '%s\n' "$SAVED_PID" >"$PID_FILE"
}

choose_free_port() {
  PORT=$BASE_PORT
  while [ "$PORT" -le "$MAX_PORT" ]; do
    if ! lsof -nP -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1; then
      return 0
    fi
    PORT=$((PORT + 1))
  done
  echo "No free local port was found between $BASE_PORT and $MAX_PORT."
  exit 1
}

echo "Project directory: $PROJECT_DIR"

if ! command -v npm >/dev/null 2>&1; then
  echo "npm is not installed. Please install Node.js first."
  exit 1
fi

if [ ! -d "$PROJECT_DIR/node_modules" ]; then
  echo "node_modules not found. Installing the locked dependencies..."
  npm ci --prefix "$PROJECT_DIR"
fi

if saved_server_ready; then
  echo "This checkout's Astro dev server is already running."
else
  choose_free_port
  URL="http://127.0.0.1:${PORT}/IRSENSORCAR/"
  echo "Starting Astro dev server..."
  nohup npm --prefix "$PROJECT_DIR" run dev -- --host 127.0.0.1 --port "$PORT" \
    >"$LOG_FILE" 2>&1 </dev/null &
  SERVER_PID=$!
  printf '%s\n' "$SERVER_PID" >"$PID_FILE"
  printf '%s\n' "$PORT" >"$PORT_FILE"

  for _ in {1..60}; do
    if homepage_ready; then
      echo "Astro dev server started (PID: $SERVER_PID)."
      break
    fi
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
      echo "Astro dev server exited before the homepage became ready."
      tail -40 "$LOG_FILE" 2>/dev/null || true
      exit 1
    fi
    sleep 1
  done

  if ! homepage_ready; then
    echo "The homepage did not become ready in time."
    echo "Check the log: $LOG_FILE"
    exit 1
  fi

  LISTENER_PID="$(lsof -t -nP -iTCP:"$PORT" -sTCP:LISTEN 2>/dev/null | sed -n '1p')"
  if [ -n "$LISTENER_PID" ]; then
    printf '%s\n' "$LISTENER_PID" >"$PID_FILE"
    SERVER_PID=$LISTENER_PID
  fi
fi

open "$URL"
echo "Opened: $URL"
echo "Dev server log: $LOG_FILE"
