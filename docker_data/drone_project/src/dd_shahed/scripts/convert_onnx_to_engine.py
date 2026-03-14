#!/usr/bin/env python3
"""
Convert ONNX model to TensorRT engine for x86_64 platform (RTX 4090, etc.)
This script must be run on the target platform where the engine will be used.
"""

import tensorrt as trt
import sys
import os

def build_engine(onnx_file_path, engine_file_path, fp16_mode=True, max_batch_size=1):
    """
    Build TensorRT engine from ONNX model

    Args:
        onnx_file_path: Path to ONNX model
        engine_file_path: Path where to save the engine
        fp16_mode: Enable FP16 precision (default True for RTX 4090)
        max_batch_size: Maximum batch size (default 1)
    """

    # Create logger
    TRT_LOGGER = trt.Logger(trt.Logger.INFO)

    # Create builder
    builder = trt.Builder(TRT_LOGGER)

    # Create network with explicit batch flag
    network_flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(network_flags)

    # Create ONNX parser
    parser = trt.OnnxParser(network, TRT_LOGGER)

    # Parse ONNX model
    print(f"[INFO] Loading ONNX file: {onnx_file_path}")
    with open(onnx_file_path, 'rb') as model:
        if not parser.parse(model.read()):
            print('[ERROR] Failed to parse ONNX file')
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            return None

    print(f"[INFO] ONNX model loaded successfully")
    print(f"[INFO] Network inputs: {network.num_inputs}")
    print(f"[INFO] Network outputs: {network.num_outputs}")

    # Print input/output info
    for i in range(network.num_inputs):
        input_tensor = network.get_input(i)
        print(f"  Input {i}: {input_tensor.name} - shape: {input_tensor.shape}")

    for i in range(network.num_outputs):
        output_tensor = network.get_output(i)
        print(f"  Output {i}: {output_tensor.name} - shape: {output_tensor.shape}")

    # Create builder config
    config = builder.create_builder_config()

    # Set max workspace size (8GB for RTX 4090)
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 8 << 30)

    # Enable FP16 mode if requested and supported
    if fp16_mode:
        if builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            print("[INFO] FP16 mode enabled")
        else:
            print("[WARN] FP16 not supported on this platform, using FP32")

    # Build engine
    print("[INFO] Building TensorRT engine... This may take several minutes.")
    serialized_engine = builder.build_serialized_network(network, config)

    if serialized_engine is None:
        print("[ERROR] Failed to build engine")
        return None

    # Save engine to file
    print(f"[INFO] Saving engine to: {engine_file_path}")
    with open(engine_file_path, 'wb') as f:
        f.write(serialized_engine)

    print(f"[SUCCESS] Engine saved successfully!")
    print(f"[INFO] Engine file size: {os.path.getsize(engine_file_path) / (1024*1024):.2f} MB")

    return serialized_engine


def main():
    # Default paths
    onnx_path = "./models/yolov11-shashed-model-weights-v1-640.onnx"
    engine_path = "./models/yolov11_shahed_640s_fp16_x86.engine"

    # Check if custom paths provided
    if len(sys.argv) > 1:
        onnx_path = sys.argv[1]
    if len(sys.argv) > 2:
        engine_path = sys.argv[2]

    # Check if ONNX file exists
    if not os.path.exists(onnx_path):
        print(f"[ERROR] ONNX file not found: {onnx_path}")
        print("\nUsage:")
        print(f"  {sys.argv[0]} [onnx_path] [engine_path]")
        print("\nDefault:")
        print(f"  {sys.argv[0]} {onnx_path} {engine_path}")
        return 1

    # Create models directory if it doesn't exist
    os.makedirs(os.path.dirname(engine_path), exist_ok=True)

    # Build engine
    engine = build_engine(onnx_path, engine_path, fp16_mode=True)

    if engine is None:
        return 1

    print("\n" + "="*60)
    print("CONVERSION COMPLETE!")
    print("="*60)
    print(f"ONNX model: {onnx_path}")
    print(f"Engine file: {engine_path}")
    print("\nYou can now run inference with:")
    print(f"  ./bin/yolo11_640s_fp16_infer {engine_path} ./test_data/one_shahed_blue_sky.jpg 0.25")
    print("="*60)

    return 0


if __name__ == "__main__":
    sys.exit(main())
