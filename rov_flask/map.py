def linear_map(value, from_low, from_high, to_low, to_high):
    """
    Linearly maps a value from one range to another range.
    
    Args:
        value (float): The value to be mapped.
        from_low (float): The lower bound of the original range.
        from_high (float): The upper bound of the original range.
        to_low (float): The lower bound of the target range.
        to_high (float): The upper bound of the target range.
        
    Returns:
        float: The mapped value.
    """
    # Ensure the value is within the original range
    value = max(min(value, from_high), from_low)
    
    # Map the value to the target range
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

# Example usage
original_value = 0.1
original_low = -50
original_high = 0
target_low = 100
target_high = 0

mapped_value = linear_map(original_value, original_low, original_high, target_low, target_high)
print("Mapped value:", mapped_value)
