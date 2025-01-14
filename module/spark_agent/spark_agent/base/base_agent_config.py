from abc import ABC, abstractmethod
from enum import IntEnum, Enum
import inspect

class BaseAgentConfig:
    
    def __init__(self) -> None:
        
        super().__init__()
        
        """ Initializes all member classes recursively. Ignores all names starting with '__' (buit-in methods)."""
        self.init_member_classes(self)
    
    # taken from git@github.com:leggedrobotics/legged_gym.git
    @staticmethod
    def init_member_classes(obj):
        print(f"Initializing {obj.__class__.__name__}")
        
        # Iterate over all attributes names
        for key in dir(obj):
            # Disregard builtin attributes
            if key == "__class__":
                continue
            
            # Get the corresponding attribute object
            var = getattr(obj, key)
            
            # Check if the attribute is a class but not an Enum
            if inspect.isclass(var) and not issubclass(var, Enum):
                
                # Instantiate the class
                i_var = var()
                # Set the attribute to the instance instead of the type
                setattr(obj, key, i_var)
                # Recursively init members of the attribute
                BaseAgentConfig.init_member_classes(i_var)