#!/bin/bash
# Comprehensive checkpoint tests - all configurations

echo "================================================"
echo "COMPREHENSIVE CHECKPOINT TESTS"
echo "================================================"
echo ""

# PENDULUM - RRT
echo "PENDULUM with RRT"
echo "================="
for t in 3 5 10; do
    tid=$([[ $t == 3 ]] && echo "1" || [[ $t == 5 ]] && echo "2" || echo "3")
    echo "Pendulum RRT τ=$t..."
    echo -e "1\n${tid}\n1" | ./Project4Pendulum > pend_rrt_t${t}.log 2>&1
    [ -f "pendulum_path.txt" ] && mv pendulum_path.txt pend_rrt_t${t}.txt && echo "✓"
done
echo ""

# PENDULUM - KPIECE
echo "PENDULUM with KPIECE"
echo "===================="
for t in 3 5 10; do
    tid=$([[ $t == 3 ]] && echo "1" || [[ $t == 5 ]] && echo "2" || echo "3")
    echo "Pendulum KPIECE τ=$t..."
    echo -e "1\n${tid}\n2" | ./Project4Pendulum > pend_kpiece_t${t}.log 2>&1
    [ -f "pendulum_path.txt" ] && mv pendulum_path.txt pend_kpiece_t${t}.txt && echo "✓"
done
echo ""

# CAR - RRT (configs 1, 3, 5, 7, 9)
echo "CAR with RRT"
echo "============"
for c in 1 3 5 7 9; do
    echo "Car RRT config=$c..."
    echo -e "1\n${c}\n1" | ./Project4Car > car_rrt_c${c}.log 2>&1
    [ -f "car_path.txt" ] && mv car_path.txt car_rrt_c${c}.txt && echo "✓"
done
echo ""

# CAR - KPIECE (configs 1, 3, 5, 7, 9)
echo "CAR with KPIECE"
echo "==============="
for c in 1 3 5 7 9; do
    echo "Car KPIECE config=$c..."
    echo -e "1\n${c}\n2" | ./Project4Car > car_kpiece_c${c}.log 2>&1
    [ -f "car_path.txt" ] && mv car_path.txt car_kpiece_c${c}.txt && echo "✓"
done
echo ""

echo "================================================"
echo "DONE! Generated files:"
ls pend_*_t*.txt car_*_c*.txt 2>/dev/null | wc -l | xargs echo
echo "================================================"
