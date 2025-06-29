#!/usr/bin/env python3
"""
Careful Repository Cleanup Script

This script analyzes the repository and removes only files that are safe to delete.
It preserves important files like demos/, core configuration, and documentation.
"""

import os
import shutil
import glob
from pathlib import Path
import re

class RepositoryCleaner:
    def __init__(self, repo_root="."):
        self.repo_root = Path(repo_root).resolve()
        self.removed_files = []
        self.removed_dirs = []
        self.errors = []
        
    def log_action(self, action, path, reason=""):
        """Log an action for reporting"""
        print(f"{action}: {path}")
        if reason:
            print(f"  Reason: {reason}")
    
    def check_file_usage(self, file_path):
        """Check if a file is referenced anywhere in the codebase"""
        if not file_path.exists():
            return False
            
        # Read file content
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
        except:
            return False
            
        # Search for references to this file
        file_name = file_path.name
        file_stem = file_path.stem
        
        # Search in the entire repository
        for root, dirs, files in os.walk(self.repo_root):
            for file in files:
                if file.endswith(('.py', '.md', '.sh', '.yaml', '.yml', '.txt')):
                    try:
                        with open(Path(root) / file, 'r', encoding='utf-8') as f:
                            file_content = f.read()
                            if file_name in file_content or file_stem in file_content:
                                return True
                    except:
                        continue
        return False
    
    def safe_remove_file(self, file_path, reason=""):
        """Safely remove a file if it exists and is not referenced"""
        file_path = Path(file_path)
        if not file_path.exists():
            return
            
        # Check if file is referenced
        if self.check_file_usage(file_path):
            print(f"⚠️  SKIPPING {file_path} - File is referenced in codebase")
            return
            
        try:
            file_path.unlink()
            self.removed_files.append(str(file_path))
            self.log_action("🗑️  REMOVED", file_path, reason)
        except Exception as e:
            self.errors.append(f"Error removing {file_path}: {e}")
    
    def safe_remove_dir(self, dir_path, reason=""):
        """Safely remove a directory if it exists and is empty or contains only safe files"""
        dir_path = Path(dir_path)
        if not dir_path.exists():
            return
            
        # Check if directory contains important files
        if self.has_important_files(dir_path):
            print(f"⚠️  SKIPPING {dir_path} - Contains important files")
            return
            
        try:
            shutil.rmtree(dir_path)
            self.removed_dirs.append(str(dir_path))
            self.log_action("🗑️  REMOVED DIR", dir_path, reason)
        except Exception as e:
            self.errors.append(f"Error removing directory {dir_path}: {e}")
    
    def has_important_files(self, dir_path):
        """Check if directory contains important files that should not be deleted"""
        important_patterns = [
            '*.py', '*.md', '*.yaml', '*.yml', '*.sh', '*.txt',
            '*.srv', '*.msg', '*.action', '*.urdf', '*.world'
        ]
        
        for pattern in important_patterns:
            if list(dir_path.glob(pattern)):
                return True
        return False
    
    def cleanup_old_logs(self):
        """Remove old log files (keep recent ones)"""
        log_dir = self.repo_root / "log"
        if not log_dir.exists():
            return
            
        # Keep only the latest log directory
        log_dirs = [d for d in log_dir.iterdir() if d.is_dir() and d.name != "latest"]
        log_dirs.sort(key=lambda x: x.stat().st_mtime, reverse=True)
        
        # Remove old log directories (keep the 3 most recent)
        for old_dir in log_dirs[3:]:
            try:
                shutil.rmtree(old_dir)
                self.removed_dirs.append(str(old_dir))
                self.log_action("🗑️  REMOVED OLD LOG", old_dir, "Keeping only 3 most recent")
            except Exception as e:
                self.errors.append(f"Error removing old log {old_dir}: {e}")
    
    def cleanup_cache_files(self):
        """Remove cache files and directories"""
        cache_patterns = [
            "**/__pycache__",
            "**/*.pyc",
            "**/*.pyo",
            "**/.DS_Store",
            "**/Thumbs.db"
        ]
        
        for pattern in cache_patterns:
            for cache_path in self.repo_root.glob(pattern):
                if cache_path.is_file():
                    self.safe_remove_file(cache_path, "Cache file")
                elif cache_path.is_dir():
                    self.safe_remove_dir(cache_path, "Cache directory")
    
    def cleanup_old_build_files(self):
        """Remove old build files"""
        # Remove old build directories outside ros2_workspace
        old_build_dirs = [
            "build",
            "install"
        ]
        
        for build_dir in old_build_dirs:
            build_path = self.repo_root / build_dir
            if build_path.exists():
                self.safe_remove_dir(build_path, "Old build directory")
    
    def cleanup_old_scripts(self):
        """Remove old scripts that are no longer needed"""
        old_scripts = [
            "scripts/launch_swarm.sh",
            "scripts/setup_macos.sh", 
            "scripts/setup_project.sh"
        ]
        
        for script in old_scripts:
            script_path = self.repo_root / script
            if script_path.exists():
                # Check if script is referenced in documentation
                if not self.check_file_usage(script_path):
                    self.safe_remove_file(script_path, "Old script no longer referenced")
    
    def cleanup_old_launch_files(self):
        """Remove old launch files that are no longer used"""
        old_launch_files = [
            "launch/enhanced_swarm_system.launch.py",
            "launch/formation_control.launch.py", 
            "launch/gazebo_swarm_simulation.launch.py"
        ]
        
        for launch_file in old_launch_files:
            launch_path = self.repo_root / launch_file
            if launch_path.exists():
                # Check if launch file is referenced
                if not self.check_file_usage(launch_path):
                    self.safe_remove_file(launch_path, "Old launch file no longer referenced")
    
    def cleanup_old_source_files(self):
        """Remove old source files that have been integrated into ROS2 workspace"""
        old_source_dirs = [
            "src/control",
            "src/environments", 
            "src/rl",
            "src/scenarios",
            "src/simulation"
        ]
        
        for source_dir in old_source_dirs:
            source_path = self.repo_root / source_dir
            if source_path.exists():
                # Check if directory contains important files
                if not self.has_important_files(source_path):
                    self.safe_remove_dir(source_path, "Old source directory integrated into ROS2")
    
    def cleanup_old_service_files(self):
        """Remove old service files that are now in ROS2 workspace"""
        old_service_files = [
            "srv/AddObstacle.srv",
            "srv/SetController.srv",
            "srv/SetFormation.srv"
        ]
        
        for service_file in old_service_files:
            service_path = self.repo_root / service_file
            if service_path.exists():
                if not self.check_file_usage(service_path):
                    self.safe_remove_file(service_path, "Service definition moved to ROS2 workspace")
    
    def cleanup_old_urdf_world_files(self):
        """Remove old URDF and world files that are now in ROS2 workspace"""
        old_files = [
            "urdf/swarm_robot.urdf.xacro",
            "worlds/swarm_arena.world"
        ]
        
        for file_path in old_files:
            path = self.repo_root / file_path
            if path.exists():
                if not self.check_file_usage(path):
                    self.safe_remove_file(path, "File moved to ROS2 workspace")
    
    def cleanup_empty_directories(self):
        """Remove empty directories"""
        empty_dirs = [
            "configs",
            "datasets", 
            "notebooks",
            "tests/integration",
            "tests/unit"
        ]
        
        for dir_name in empty_dirs:
            dir_path = self.repo_root / dir_name
            if dir_path.exists():
                # Check if directory is truly empty or contains only cache files
                has_files = False
                for item in dir_path.rglob("*"):
                    if item.is_file() and not item.name.startswith('.') and not item.suffix in ['.pyc', '.pyo']:
                        has_files = True
                        break
                
                if not has_files:
                    self.safe_remove_dir(dir_path, "Empty directory")
    
    def cleanup_old_model_files(self):
        """Remove old RL model files that are not used"""
        model_files = [
            "models/rl_follower_1.pkl",
            "models/rl_follower_2.pkl", 
            "models/rl_follower_3.pkl"
        ]
        
        for model_file in model_files:
            model_path = self.repo_root / model_file
            if model_path.exists():
                if not self.check_file_usage(model_path):
                    self.safe_remove_file(model_path, "RL model not used in current implementation")
    
    def cleanup_old_documentation(self):
        """Remove old documentation files that are replaced"""
        old_docs = [
            "docs/ARCHITECTURE.md"
        ]
        
        for doc_file in old_docs:
            doc_path = self.repo_root / doc_file
            if doc_path.exists():
                if not self.check_file_usage(doc_path):
                    self.safe_remove_file(doc_path, "Architecture documented in main README")
    
    def cleanup_old_frame_files(self):
        """Remove old frame visualization files"""
        frame_files = list(self.repo_root.glob("frames_*.gv")) + list(self.repo_root.glob("frames_*.pdf"))
        
        for frame_file in frame_files:
            if not self.check_file_usage(frame_file):
                self.safe_remove_file(frame_file, "Old frame visualization file")
    
    def cleanup_old_test_files(self):
        """Remove old test files that are no longer needed"""
        old_test_files = [
            "test_all_demos.py"
        ]
        
        for test_file in old_test_files:
            test_path = self.repo_root / test_file
            if test_path.exists():
                if not self.check_file_usage(test_path):
                    self.safe_remove_file(test_path, "Old test file no longer needed")
    
    def cleanup_old_requirements_files(self):
        """Remove old requirements files that are no longer needed"""
        old_req_files = [
            "requirements-conda.txt"
        ]
        
        for req_file in old_req_files:
            req_path = self.repo_root / req_file
            if req_path.exists():
                if not self.check_file_usage(req_path):
                    self.safe_remove_file(req_path, "Old requirements file no longer needed")
    
    def run_cleanup(self):
        """Run the complete cleanup process"""
        print("🧹 Starting careful repository cleanup...")
        print("=" * 60)
        
        # Run all cleanup operations
        self.cleanup_cache_files()
        self.cleanup_old_logs()
        self.cleanup_old_build_files()
        self.cleanup_old_scripts()
        self.cleanup_old_launch_files()
        self.cleanup_old_source_files()
        self.cleanup_old_service_files()
        self.cleanup_old_urdf_world_files()
        self.cleanup_empty_directories()
        self.cleanup_old_model_files()
        self.cleanup_old_documentation()
        self.cleanup_old_frame_files()
        self.cleanup_old_test_files()
        self.cleanup_old_requirements_files()
        
        # Print summary
        print("\n" + "=" * 60)
        print("📊 CLEANUP SUMMARY")
        print("=" * 60)
        print(f"Files removed: {len(self.removed_files)}")
        print(f"Directories removed: {len(self.removed_dirs)}")
        print(f"Errors encountered: {len(self.errors)}")
        
        if self.removed_files:
            print("\n🗑️  Removed Files:")
            for file in self.removed_files:
                print(f"  - {file}")
        
        if self.removed_dirs:
            print("\n🗑️  Removed Directories:")
            for dir_path in self.removed_dirs:
                print(f"  - {dir_path}")
        
        if self.errors:
            print("\n❌ Errors:")
            for error in self.errors:
                print(f"  - {error}")
        
        print("\n✅ Cleanup completed!")
        print("📁 Preserved important directories:")
        print("  - demos/ (contains core swarm logic)")
        print("  - ros2_workspace/ (main ROS2 implementation)")
        print("  - config/ (configuration files)")
        print("  - docker/ (Docker support)")
        print("  - Documentation files (README, etc.)")

def main():
    """Main function"""
    cleaner = RepositoryCleaner()
    cleaner.run_cleanup()

if __name__ == "__main__":
    main() 