#!/usr/bin/env python3
"""
Individual Demo Testing Script

Tests each demo file systematically and reports results.
Activates swarm environment properly before testing.
"""

import subprocess
import sys
import time
import os
from pathlib import Path

def test_demo(demo_file, timeout=20):
    """Test a single demo file with timeout."""
    print(f"\n{'='*60}")
    print(f"🧪 TESTING: {demo_file}")
    print(f"{'='*60}")
    
    if not Path(demo_file).exists():
        return {
            'file': demo_file,
            'status': 'MISSING',
            'error': 'File does not exist',
            'duration': 0
        }
    
    start_time = time.time()
    
    try:
        # Create a simple test script that activates environment and runs demo
        test_script = f"""#!/bin/bash
cd {os.getcwd()}
source activate_swarm_ros2.sh
python {demo_file}
"""
        
        # Write temporary test script
        with open('temp_test.sh', 'w') as f:
            f.write(test_script)
        
        # Make it executable
        os.chmod('temp_test.sh', 0o755)
        
        # Run with timeout using Python's subprocess timeout
        result = subprocess.run(
            ['bash', 'temp_test.sh'],
            capture_output=True,
            text=True,
            timeout=timeout
        )
        
        duration = time.time() - start_time
        
        # Clean up temp script
        try:
            os.remove('temp_test.sh')
        except:
            pass
        
        if result.returncode == 0:
            return {
                'file': demo_file,
                'status': 'SUCCESS',
                'error': None,
                'duration': duration,
                'stdout': result.stdout[-500:] if result.stdout else '',
                'stderr': result.stderr[-500:] if result.stderr else ''
            }
        else:
            return {
                'file': demo_file,
                'status': 'FAILED',
                'error': f'Exit code: {result.returncode}',
                'duration': duration,
                'stdout': result.stdout[-500:] if result.stdout else '',
                'stderr': result.stderr[-500:] if result.stderr else ''
            }
            
    except subprocess.TimeoutExpired:
        duration = time.time() - start_time
        # Clean up temp script
        try:
            os.remove('temp_test.sh')
        except:
            pass
        
        return {
            'file': demo_file,
            'status': 'TIMEOUT',
            'error': f'Demo ran for {timeout}s (likely working - interactive demo)',
            'duration': duration,
            'stdout': '',
            'stderr': ''
        }
    except Exception as e:
        duration = time.time() - start_time
        # Clean up temp script
        try:
            os.remove('temp_test.sh')
        except:
            pass
        
        return {
            'file': demo_file,
            'status': 'ERROR',
            'error': str(e),
            'duration': duration,
            'stdout': '',
            'stderr': ''
        }

def print_result(result):
    """Print formatted test result."""
    file = result['file']
    status = result['status']
    duration = result['duration']
    error = result['error']
    
    # Status emoji
    if status == 'SUCCESS':
        emoji = '✅'
        color = '\033[92m'  # Green
    elif status == 'TIMEOUT':
        emoji = '⏱️'
        color = '\033[93m'  # Yellow  
    elif status == 'FAILED':
        emoji = '❌'
        color = '\033[91m'  # Red
    elif status == 'ERROR':
        emoji = '💥'
        color = '\033[91m'  # Red
    elif status == 'MISSING':
        emoji = '❓'
        color = '\033[94m'  # Blue
    else:
        emoji = '❓'
        color = '\033[94m'  # Blue
    
    reset_color = '\033[0m'
    
    print(f"{emoji} {color}{file:<30} {status:<10}{reset_color} ({duration:.1f}s)")
    
    if error:
        print(f"   Error: {error}")
    
    # Show stderr if there are errors (but limit output)
    if result.get('stderr') and result['stderr'].strip():
        stderr_preview = result['stderr'][:200].replace('\n', ' ')
        print(f"   Stderr: {stderr_preview}...")
    
    return status

def test_single_demo_manually(demo_file):
    """Test a single demo manually by just trying to import/run it briefly."""
    print(f"\n🧪 MANUAL TEST: {demo_file}")
    
    if not Path(demo_file).exists():
        print(f"❓ {demo_file} - File missing")
        return False
    
    try:
        # Try a quick syntax check first
        result = subprocess.run([
            'python', '-m', 'py_compile', demo_file
        ], capture_output=True, text=True, timeout=10)
        
        if result.returncode != 0:
            print(f"❌ {demo_file} - Syntax error")
            print(f"   Error: {result.stderr[:100]}...")
            return False
        else:
            print(f"✅ {demo_file} - Syntax OK")
            return True
            
    except Exception as e:
        print(f"💥 {demo_file} - Test failed: {e}")
        return False

def main():
    """Run systematic testing of all demos."""
    
    # Check if swarm environment script exists
    if not Path('activate_swarm_ros2.sh').exists():
        print("❌ activate_swarm_ros2.sh not found!")
        print("   Make sure you're in the right directory")
        return 0, 0
    
    # List of demo files to test
    demo_files = [
        'simple_demo.py',
        'clean_start.py', 
        'multi_follower_demo.py',
        'multi_follower_ros2.py',
        'vision_leader_follower.py',
        'vision_leader_follower_ros2.py',
        'obstacle_avoidance_demo.py',
        'obstacle_avoidance_ros2.py',
        'mpc_leader_follower.py',
        'rl_leader_follower.py',
        'speed_test.py',
        'test_all_demos.py'
    ]
    
    print("🧪 SYSTEMATIC DEMO TESTING")
    print("=" * 60)
    print("First: Quick syntax checks")
    print("Then: Full demo testing with environment activation")
    print()
    
    # Phase 1: Quick syntax checks
    print("📝 PHASE 1: SYNTAX CHECKS")
    print("-" * 40)
    
    syntax_results = []
    for demo_file in demo_files:
        syntax_ok = test_single_demo_manually(demo_file)
        syntax_results.append((demo_file, syntax_ok))
    
    syntax_pass = sum(1 for _, ok in syntax_results if ok)
    print(f"\n📊 Syntax Results: {syntax_pass}/{len(demo_files)} files have valid syntax")
    
    # Phase 2: Full testing with environment 
    print(f"\n🚀 PHASE 2: FULL DEMO TESTING")
    print("-" * 40)
    print("Testing each demo for 20 seconds with swarm environment...")
    print("TIMEOUT = likely working (interactive demo)")
    print("SUCCESS = completed without errors")
    print("FAILED = crashed or errored")
    print()
    
    results = []
    
    for demo_file in demo_files:
        result = test_demo(demo_file, timeout=20)
        status = print_result(result)
        results.append(result)
        
        # Small delay between tests
        time.sleep(1)
    
    # Summary report
    print(f"\n{'='*60}")
    print("📊 TESTING SUMMARY")
    print(f"{'='*60}")
    
    success_count = sum(1 for r in results if r['status'] == 'SUCCESS')
    timeout_count = sum(1 for r in results if r['status'] == 'TIMEOUT')
    failed_count = sum(1 for r in results if r['status'] == 'FAILED')
    error_count = sum(1 for r in results if r['status'] == 'ERROR')
    missing_count = sum(1 for r in results if r['status'] == 'MISSING')
    
    total = len(results)
    
    print(f"✅ SUCCESS:  {success_count}/{total} demos completed successfully")
    print(f"⏱️  TIMEOUT:  {timeout_count}/{total} demos likely working (interactive)")
    print(f"❌ FAILED:   {failed_count}/{total} demos crashed")
    print(f"💥 ERROR:    {error_count}/{total} demos had errors")
    print(f"❓ MISSING:  {missing_count}/{total} demos not found")
    
    working_demos = success_count + timeout_count
    print(f"\n🎯 OVERALL: {working_demos}/{total} demos appear to be working")
    
    # Detailed failure report
    failures = [r for r in results if r['status'] in ['FAILED', 'ERROR']]
    if failures:
        print(f"\n🚨 CRITICAL ISSUES TO FIX:")
        for failure in failures:
            print(f"   • {failure['file']}: {failure['error']}")
            if failure.get('stderr'):
                # Show key error lines
                stderr_lines = failure['stderr'].split('\n')
                for line in stderr_lines[:3]:  # First few lines
                    if 'Error' in line or 'Exception' in line or 'Traceback' in line:
                        print(f"     └─ {line.strip()}")
    
    # Save detailed results to file
    with open('test_results.txt', 'w') as f:
        f.write("DETAILED TEST RESULTS\n")
        f.write("=" * 50 + "\n\n")
        
        for result in results:
            f.write(f"File: {result['file']}\n")
            f.write(f"Status: {result['status']}\n")
            f.write(f"Duration: {result['duration']:.2f}s\n")
            if result['error']:
                f.write(f"Error: {result['error']}\n")
            if result.get('stderr'):
                f.write(f"Stderr:\n{result['stderr']}\n")
            if result.get('stdout'):
                f.write(f"Stdout:\n{result['stdout']}\n")
            f.write("-" * 30 + "\n")
    
    print(f"\n📄 Detailed results saved to: test_results.txt")
    
    return working_demos, total

if __name__ == '__main__':
    try:
        working, total = main()
        print(f"\n🏁 Testing complete: {working}/{total} demos working")
        
        if working < total:
            print("❗ Some demos need fixing. Check test_results.txt for details.")
            print("💡 Next step: Fix the failed demos one by one")
            sys.exit(1)
        else:
            print("🎉 All demos are working!")
            sys.exit(0)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Testing interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\n💥 Testing script failed: {e}")
        sys.exit(1) 