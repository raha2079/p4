#!/bin/bash
# Prepare Project 4 Checkpoint Submission
# This script helps create the ZIP file for Canvas submission

echo "========================================="
echo "Project 4 Checkpoint Submission Prep"
echo "========================================="
echo ""

# Get NetID from user
read -p "Enter your NetID (e.g., abc123): " netid

if [ -z "$netid" ]; then
    echo "Error: NetID cannot be empty!"
    exit 1
fi

echo ""
echo "Using NetID: $netid"
echo ""

# Clean build files
echo "Step 1: Cleaning build files..."
make clean > /dev/null 2>&1
echo "✓ Build files cleaned"
echo ""

# Go to parent directory
cd ..

# Create ZIP file
zipfile="Project4_Checkpoint_${netid}.zip"
echo "Step 2: Creating ZIP file: $zipfile"

zip -r "$zipfile" template/ \
    -x "template/build/*" \
    -x "template/.DS_Store" \
    -x "template/*/.DS_Store" \
    -x "template/__pycache__/*" \
    -x "template/*.log" \
    -x "template/.*" \
    > /dev/null 2>&1

if [ -f "$zipfile" ]; then
    filesize=$(ls -lh "$zipfile" | awk '{print $5}')
    echo "✓ ZIP file created: $zipfile ($filesize)"
else
    echo "✗ Failed to create ZIP file!"
    exit 1
fi

echo ""
echo "========================================="
echo "Submission files ready!"
echo "========================================="
echo ""
echo "📦 Files to submit to Canvas:"
echo "   1. $zipfile"
echo "   2. Checkpoint_Report_FINAL.pdf (create from HTML - see instructions)"
echo ""
echo "📝 Next steps:"
echo "   1. Open Checkpoint_Report_FINAL.html in browser"
echo "   2. Print to PDF (Cmd+P > Save as PDF)"
echo "   3. Upload ZIP and PDF to Canvas"
echo "   4. Deadline: TODAY at 1pm!"
echo ""
echo "📄 See PDF_CONVERSION_INSTRUCTIONS.txt for detailed steps"
echo ""
echo "✓ Preparation complete!"
