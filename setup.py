from setuptools import setup, find_packages

setup(name='task_planner',
      version='1.0.1',
      description='A planning library',
      url='https://github.com/SLE-LL-95/task-planner',
      author='Alex Mitrevski, Lou Lauter (Panda Interface)',
      author_email='aleksandar.mitrevski@h-brs.de',
      keywords='robotics task_planning',
      packages=find_packages(exclude=['contrib', 'docs', 'tests']),
      project_urls={
          'Source': 'https://github.com/ropod-project/task-planning'
      })
