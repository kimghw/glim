#!/bin/bash

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'
BOLD='\033[1m'

# Source directory
SOURCE_DIR="/home/kimghw/glim/rosbag_data"

echo -e "${BOLD}${CYAN}======================================"
echo "     ROS Bag Data Transfer Tool"
echo "======================================${NC}"
echo ""

# Check source directory
if [ ! -d "$SOURCE_DIR" ]; then
    echo -e "${RED}Error: Source directory not found: $SOURCE_DIR${NC}"
    exit 1
fi

# Calculate total size
TOTAL_SIZE=$(du -sh "$SOURCE_DIR" | cut -f1)
echo -e "${BLUE}Total rosbag data size:${NC} ${BOLD}$TOTAL_SIZE${NC}"

# Count rosbag folders
NUM_BAGS=$(ls -d $SOURCE_DIR/rosbag_* 2>/dev/null | wc -l)
echo -e "${BLUE}Number of rosbags:${NC} ${BOLD}$NUM_BAGS${NC}"
echo ""

# Show available storage devices
echo -e "${YELLOW}Available Storage Devices:${NC}"
echo "======================================"
df -h | grep -E "Filesystem|/media|/mnt" | while IFS= read -r line; do
    echo "$line"
done
echo ""

# Suggest best option
echo -e "${GREEN}Recommended External Storage:${NC}"
echo "  • T9 Drive (/media/kimghw/T9) - 1.8TB free"
echo ""

# Ask user for confirmation
echo "Select destination:"
echo "  1. T9 External Drive (/media/kimghw/T9)"
echo "  2. Internal NTFS partition (/media/kimghw/6A7A44C37A448DAF)"
echo "  3. Custom path"
echo "  4. Cancel"
echo ""
read -p "Your choice [1-4]: " choice

case $choice in
    1)
        DEST_BASE="/media/kimghw/T9"
        ;;
    2)
        DEST_BASE="/media/kimghw/6A7A44C37A448DAF"
        ;;
    3)
        read -p "Enter destination path: " DEST_BASE
        ;;
    4)
        echo -e "${YELLOW}Operation cancelled${NC}"
        exit 0
        ;;
    *)
        echo -e "${RED}Invalid choice${NC}"
        exit 1
        ;;
esac

# Check if destination exists
if [ ! -d "$DEST_BASE" ]; then
    echo -e "${RED}Error: Destination doesn't exist: $DEST_BASE${NC}"
    exit 1
fi

# Check available space
AVAIL_SPACE=$(df -h "$DEST_BASE" | tail -1 | awk '{print $4}')
echo -e "\n${BLUE}Available space at destination:${NC} ${BOLD}$AVAIL_SPACE${NC}"

# Create destination directory
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
DEST_DIR="$DEST_BASE/rosbag_backup_$TIMESTAMP"

echo ""
echo -e "${YELLOW}Transfer Options:${NC}"
echo "  1. Move (faster, removes original)"
echo "  2. Copy (keeps original)"
echo "  3. Copy with verification (safest)"
echo ""
read -p "Select option [1-3]: " transfer_option

echo ""
echo -e "${CYAN}Destination:${NC} $DEST_DIR"
read -p "Proceed? [Y/n]: " confirm

if [ "$confirm" = "n" ] || [ "$confirm" = "N" ]; then
    echo -e "${YELLOW}Operation cancelled${NC}"
    exit 0
fi

# Create destination directory
mkdir -p "$DEST_DIR"

# Perform transfer
case $transfer_option in
    1)
        echo -e "\n${GREEN}Moving rosbag data...${NC}"
        mv -v "$SOURCE_DIR" "$DEST_DIR/" 2>&1 | while IFS= read -r line; do
            echo -e "  ${BLUE}→${NC} $line"
        done
        # Create symlink
        ln -s "$DEST_DIR/rosbag_data" "$SOURCE_DIR"
        echo -e "${GREEN}Created symlink: $SOURCE_DIR -> $DEST_DIR/rosbag_data${NC}"
        ;;

    2)
        echo -e "\n${GREEN}Copying rosbag data...${NC}"
        # Use rsync for better progress display
        if command -v rsync &> /dev/null; then
            rsync -avh --progress "$SOURCE_DIR" "$DEST_DIR/"
        else
            cp -rv "$SOURCE_DIR" "$DEST_DIR/" 2>&1 | while IFS= read -r line; do
                echo -e "  ${BLUE}→${NC} $line"
            done
        fi
        ;;

    3)
        echo -e "\n${GREEN}Copying with verification...${NC}"
        # Use rsync with checksum verification
        if command -v rsync &> /dev/null; then
            rsync -avhc --progress "$SOURCE_DIR" "$DEST_DIR/"

            # Verify sizes
            echo -e "\n${BLUE}Verifying transfer...${NC}"
            SOURCE_SIZE=$(du -sb "$SOURCE_DIR" | cut -f1)
            DEST_SIZE=$(du -sb "$DEST_DIR/rosbag_data" | cut -f1)

            if [ "$SOURCE_SIZE" -eq "$DEST_SIZE" ]; then
                echo -e "${GREEN}✓ Verification successful${NC}"

                # Create verification report
                cat > "$DEST_DIR/transfer_report.txt" << EOF
Transfer Report
===============
Date: $(date)
Source: $SOURCE_DIR
Destination: $DEST_DIR/rosbag_data
Total Size: $TOTAL_SIZE
Number of bags: $NUM_BAGS
Verification: PASSED
EOF
                echo -e "${GREEN}Transfer report saved: $DEST_DIR/transfer_report.txt${NC}"
            else
                echo -e "${RED}✗ Size mismatch! Please check the transfer${NC}"
            fi
        else
            cp -rv "$SOURCE_DIR" "$DEST_DIR/"
        fi
        ;;
esac

echo ""
echo -e "${GREEN}${BOLD}Transfer complete!${NC}"
echo ""
echo -e "${CYAN}Summary:${NC}"
echo "  • Source: $SOURCE_DIR"
echo "  • Destination: $DEST_DIR"
echo "  • Size: $TOTAL_SIZE"
echo ""

# Ask about cleanup
if [ "$transfer_option" != "1" ]; then
    read -p "Delete original files to free up space? [y/N]: " delete_orig
    if [ "$delete_orig" = "y" ] || [ "$delete_orig" = "Y" ]; then
        echo -e "${YELLOW}Removing original files...${NC}"
        rm -rf "$SOURCE_DIR"
        echo -e "${GREEN}Original files removed${NC}"

        # Create symlink to new location
        ln -s "$DEST_DIR/rosbag_data" "$SOURCE_DIR"
        echo -e "${GREEN}Created symlink for compatibility${NC}"
    fi
fi

echo ""
echo -e "${GREEN}Done!${NC}"