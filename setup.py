from setuptools import setup, find_packages

setup(
    name='TraffiSim',
    version='0.1.0',
    author='A. Sharan',
    description='A modular intersection traffic simulator with IDM, MOBIL, and adaptive signals',
    packages=find_packages(),
    install_requires=[
        'pygame>=2.0.0'
    ],
extras_require={
    'rendering': ['pygame>=2.0.0'],
    'analysis': [
        'streamlit',
        'pandas',
        'plotly',
        'seaborn',
        'matplotlib'
     
    ],
},
    python_requires='>=3.7',
    entry_points={
        'console_scripts': [
            'traffisim-run = traffisim.run:main_cli',  
        
        ]
    }
)
