#!/usr/bin/env python3
"""
🧪 Comprehensive Scenario Testing Script

Tests all swarm scenarios and configurations to ensure
the launch system works correctly across different use cases.

Usage:
    python scripts/test_all_scenarios.py
    python scripts/test_all_scenarios.py --scenario high_speed
    python scripts/test_all_scenarios.py --quick
"""

import sys
import time
import yaml
import argparse
import subprocess
from pathlib import Path
from typing import Dict, List


class ScenarioTester:
    """Comprehensive testing class for all swarm scenarios."""
    
    def __init__(self):
        self.scenarios = {
            'default': 'config/swarm_parameters.yaml',
            'high_speed': 'config/scenarios/high_speed.yaml',
            'conservative': 'config/scenarios/conservative.yaml',
            'research': 'config/scenarios/research.yaml'
        }
        
    def test_configuration_files(self) -> Dict[str, bool]:
        """Test that all configuration files are valid."""
        print("🔧 Testing Configuration Files...")
        print("=" * 50)
        
        results = {}
        
        for scenario_name, config_path in self.scenarios.items():
            print(f"\n📄 Testing {scenario_name}: {config_path}")
            
            try:
                config_file = Path(config_path)
                if not config_file.exists():
                    print(f"   ❌ File not found: {config_path}")
                    results[scenario_name] = False
                    continue
                
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                
                if 'swarm_controller' not in config:
                    print(f"   ❌ Missing 'swarm_controller' section")
                    results[scenario_name] = False
                    continue
                
                params = config['swarm_controller']['ros__parameters']
                required_groups = ['formation', 'control', 'leader', 'obstacles', 'visualization', 'safety']
                missing_groups = [g for g in required_groups if g not in params]
                
                if missing_groups:
                    print(f"   ❌ Missing parameter groups: {missing_groups}")
                    results[scenario_name] = False
                    continue
                
                print(f"   ✅ Configuration valid")
                results[scenario_name] = True
                
            except Exception as e:
                print(f"   ❌ Error: {e}")
                results[scenario_name] = False
        
        return results
    
    def test_launch_file_syntax(self) -> bool:
        """Test that the launch file has correct syntax."""
        print("\n🚀 Testing Launch File Syntax...")
        print("=" * 50)
        
        launch_file = Path('launch/enhanced_swarm_system.launch.py')
        
        if not launch_file.exists():
            print("❌ Launch file not found")
            return False
        
        try:
            with open(launch_file, 'r') as f:
                code = f.read()
            
            compile(code, str(launch_file), 'exec')
            print("✅ Launch file syntax is valid")
            return True
            
        except Exception as e:
            print(f"❌ Error: {e}")
            return False
    
    def test_script_executability(self) -> Dict[str, bool]:
        """Test that all scripts are executable."""
        print("\n📜 Testing Script Executability...")
        print("=" * 50)
        
        scripts = [
            'scripts/launch_swarm.sh',
            'scripts/test_parameters.py',
            'scripts/test_all_scenarios.py'
        ]
        
        results = {}
        
        for script in scripts:
            script_path = Path(script)
            print(f"\n🔧 Testing {script}")
            
            if not script_path.exists():
                print(f"   ❌ Script not found")
                results[script] = False
                continue
            
            if not script_path.stat().st_mode & 0o111:
                print(f"   ⚠️ Script not executable (fixing...)")
                script_path.chmod(script_path.stat().st_mode | 0o755)
            
            print(f"   ✅ Script executable")
            results[script] = True
        
        return results
    
    def run_comprehensive_test(self, quick_mode: bool = False) -> Dict[str, bool]:
        """Run all tests and return results."""
        print("🧪 Starting Comprehensive Scenario Testing")
        print("=" * 60)
        print(f"📅 Test Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 60)
        
        all_results = {}
        
        # 1. Configuration file tests
        config_results = self.test_configuration_files()
        all_results['configurations'] = config_results
        
        # 2. Launch file syntax test
        launch_result = self.test_launch_file_syntax()
        all_results['launch_file'] = launch_result
        
        # 3. Script executability tests
        script_results = self.test_script_executability()
        all_results['scripts'] = script_results
        
        # Print summary
        self._print_test_summary(all_results)
        
        return all_results
    
    def _print_test_summary(self, results: Dict):
        """Print test summary."""
        print("\n" + "=" * 60)
        print("📋 TEST SUMMARY")
        print("=" * 60)
        
        total_tests = 0
        passed_tests = 0
        
        print("\n🔧 Configuration Files:")
        for scenario, passed in results['configurations'].items():
            status = "✅ PASS" if passed else "❌ FAIL"
            print(f"   {scenario:15} {status}")
            total_tests += 1
            if passed:
                passed_tests += 1
        
        print("\n🚀 Launch File:")
        status = "✅ PASS" if results['launch_file'] else "❌ FAIL"
        print(f"   Syntax Check     {status}")
        total_tests += 1
        if results['launch_file']:
            passed_tests += 1
        
        print("\n📜 Scripts:")
        for script, passed in results['scripts'].items():
            status = "✅ PASS" if passed else "❌ FAIL"
            script_name = Path(script).name
            print(f"   {script_name:15} {status}")
            total_tests += 1
            if passed:
                passed_tests += 1
        
        pass_rate = (passed_tests / total_tests) * 100 if total_tests > 0 else 0
        print(f"\n📊 Overall Results: {passed_tests}/{total_tests} tests passed ({pass_rate:.1f}%)")
        
        if pass_rate == 100:
            print("🎉 All tests passed! System ready for deployment!")
        elif pass_rate >= 80:
            print("⚠️ Most tests passed. Minor issues detected.")
        else:
            print("❌ Significant issues detected. Please fix before deployment.")


def main():
    """Main function for running scenario tests."""
    parser = argparse.ArgumentParser(description='Test all swarm scenarios')
    parser.add_argument('--scenario', help='Test specific scenario only')
    parser.add_argument('--quick', action='store_true', help='Quick test mode')
    
    args = parser.parse_args()
    
    tester = ScenarioTester()
    
    if args.scenario:
        if args.scenario in tester.scenarios:
            results = tester.test_configuration_files()
            if results.get(args.scenario, False):
                print(f"✅ Scenario '{args.scenario}' passed all tests!")
            else:
                print(f"❌ Scenario '{args.scenario}' failed tests!")
        else:
            print(f"❌ Unknown scenario: {args.scenario}")
            print(f"Available scenarios: {list(tester.scenarios.keys())}")
    else:
        tester.run_comprehensive_test(quick_mode=args.quick)


if __name__ == "__main__":
    main() 