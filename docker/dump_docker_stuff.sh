#!/bin/bash

# 1. Determine paths relative to the script location
# This ensures the script works whether you run it as ./dump... or from the root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
OUTPUT_FILE="$PROJECT_ROOT/docker_context_dump.txt"

# 2. Define files to dump (Paths relative to Project Root for readability)
# We map the readable name (key) to the actual file path (value)
declare -A FILES
FILES=(
    ["Makefile"]="$PROJECT_ROOT/Makefile"
    [".dockerignore"]="$PROJECT_ROOT/.dockerignore"
    ["docker/Makefile"]="$SCRIPT_DIR/Makefile"
    ["docker/Dockerfile"]="$SCRIPT_DIR/Dockerfile"
    ["docker/docker-compose.yaml"]="$SCRIPT_DIR/docker-compose.yaml"
    ["docker/.env"]="$SCRIPT_DIR/.env"
)

# 3. Start the Dump
echo "==============================================================================" > "$OUTPUT_FILE"
echo "PROJECT DOCKER INFRASTRUCTURE DUMP" >> "$OUTPUT_FILE"
echo "Generated on: $(date)" >> "$OUTPUT_FILE"
echo "==============================================================================" >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

# 4. Tree Visualization (Excluded compiled/heavy folders)
if command -v tree &> /dev/null; then
    echo "--- [ PROJECT STRUCTURE ] ------------------------------------------------" >> "$OUTPUT_FILE"
    # Run tree on the project root
    tree "$PROJECT_ROOT" -L 2 -I 'build|install|log|cache|bags|docker-dev-build-cache' --noreport >> "$OUTPUT_FILE"
    echo "" >> "$OUTPUT_FILE"
fi

# 5. Dump Content
# We iterate through the specific files we care about
for NAME in "${!FILES[@]}"; do
    FILE_PATH="${FILES[$NAME]}"
    
    if [ -f "$FILE_PATH" ]; then
        echo "--- [ FILE: $NAME ] ---------------------------------------------------" >> "$OUTPUT_FILE"
        cat "$FILE_PATH" >> "$OUTPUT_FILE"
        echo "" >> "$OUTPUT_FILE"
        echo "" >> "$OUTPUT_FILE"
    else
        echo "⚠️  Warning: File '$NAME' not found at $FILE_PATH"
    fi
done

echo "✅ Dump complete!" 
echo "   File saved to: $OUTPUT_FILE"