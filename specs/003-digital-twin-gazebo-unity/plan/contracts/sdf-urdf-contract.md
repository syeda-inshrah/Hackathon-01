# SDF vs URDF Interface Contract

**Interface**: SDFURDFInterface
**Implementation**: Python with URDF/SDF conversion tools
**Version**: 1.0

## Purpose
This contract defines the interface for managing the relationship between SDF (Simulation Description Format) and URDF (Unified Robot Description Format), allowing for physics-optimized SDF models while maintaining ROS ecosystem compatibility through URDF.

## Methods

### convert_urdf_to_sdf(urdf_file_path, output_sdf_path)
Converts a URDF file to SDF format for Gazebo physics simulation.

**Parameters**:
- urdf_file_path (string): Path to the input URDF file
- output_sdf_path (string): Path where the output SDF file should be saved

**Returns**: ConversionResult object with success status and transformation details

**Errors**:
- InvalidURDFError: If the input URDF file is not valid
- ConversionError: If the conversion process fails

### extract_physics_properties_from_sdf(sdf_file_path)
Extracts physics properties from an SDF file for advanced Gazebo simulation.

**Parameters**:
- sdf_file_path (string): Path to the SDF file

**Returns**: PhysicsProperties object containing collision shapes, friction coefficients, joint dynamics, etc.

**Errors**:
- InvalidSDFError: If the SDF file is not valid
- PhysicsPropertiesNotFoundError: If physics properties are not found

### validate_urdf_compatibility(sdf_file_path, urdf_file_path)
Validates that the SDF and URDF files represent compatible robot models.

**Parameters**:
- sdf_file_path (string): Path to the SDF file
- urdf_file_path (string): Path to the URDF file

**Returns**: ValidationResult object with compatibility status and discrepancy report

**Errors**:
- InvalidSDFError: If the SDF file is not valid
- InvalidURDFError: If the URDF file is not valid

### generate_urdf_visual_model(sdf_file_path, output_urdf_path)
Generates a URDF file with visual-only properties from an SDF file, omitting physics properties.

**Parameters**:
- sdf_file_path (string): Path to the input SDF file
- output_urdf_path (string): Path where the output URDF file should be saved

**Returns**: GenerationResult object with success status and model information

**Errors**:
- InvalidSDFError: If the input SDF file is not valid
- GenerationError: If the URDF generation process fails

### synchronize_models(sdf_reference, urdf_reference)
Ensures that the SDF physics model and URDF representation remain synchronized.

**Parameters**:
- sdf_reference (string/SDFModel): Reference to the SDF model
- urdf_reference (string/URDFModel): Reference to the URDF model

**Returns**: Boolean indicating successful synchronization

**Errors**:
- ModelMismatchError: If the models are not properly synchronized
- InvalidReferenceError: If the model references are invalid

## Performance Requirements
- Conversion from URDF to SDF must complete within 5 seconds for typical humanoid models
- Validation of URDF compatibility must complete within 2 seconds
- Model synchronization operations should complete within 100ms
- Generated URDF files must maintain ROS ecosystem compatibility