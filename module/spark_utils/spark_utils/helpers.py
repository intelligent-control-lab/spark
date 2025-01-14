import os

def initialize_class(class_cfg, **kwargs):
    '''
    Initialize a class from a configuration object.
    Also pass any additional keyword arguments to the class constructor
    '''
    
    class_cfg_dict = class_to_dict(class_cfg)
    
    if 'class_name' not in class_cfg_dict:
        raise ValueError(f'class_name not found in {class_cfg}')

    class_name = class_cfg_dict.pop('class_name')
    
    if class_name is None:
        return None
    
    import spark_task
    import spark_policy
    import spark_safe.safe_controller as safe_controller
    import spark_safe.safe_algo as safe_algo
    import spark_agent
    import spark_robot

    for module in [spark_policy, safe_controller, spark_agent, spark_robot, spark_task, safe_algo]:
        
        if hasattr(module, class_name):
            class_name = getattr(module, class_name)

            return class_name(**{ **class_cfg_dict, **kwargs })
    
    raise ValueError(f'Class {class_name} not found in any of the modules')

def class_to_dict(obj) -> dict:
    if not  hasattr(obj,"__dict__"):
        return obj
    result = {}
    for key in dir(obj):
        if key.startswith("_"):
            continue
        element = []
        val = getattr(obj, key)
        if isinstance(val, list):
            for item in val:
                element.append(class_to_dict(item))
        else:
            element = class_to_dict(val)
        result[key] = element
    return result

def update_class_attributes(obj, attrs: dict):
    
    for key, value in attrs.items():
        if not hasattr(obj, key):
            print(f'Attribute {key} not found in {obj}, adding to {obj}')
        setattr(obj, key, value)
    
    return obj