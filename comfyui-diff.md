PS C:\Users\hoang> Invoke-WebRequest `
>>   "https://raw.githubusercontent.com/nhatxart/RoboticsandAI---Fall-2024/refs/heads/main/comfyui-diff.md" `
>>   -OutFile "$env:TEMP\comfyui-diff.md"
PS C:\Users\hoang> Select-String `
>>   -Path "$env:TEMP\comfyui-diff.md" `
>>   -Pattern '/opt/venv|site-packages|torch|cuda|nvidia|rocm|hip|triton' `
>>   -CaseSensitive:$false |
>>   Select-Object -First 500

AppData\Local\Temp\comfyui-diff.md:45:rocm/comfyui:comfyui-0.18.2.amd0_rocm7.2.0_ubuntu24.04   c88f20157d88
67.6GB             0B
AppData\Local\Temp\comfyui-diff.md:46:PS C:\Users\hoang> wsl docker run --rm --device=/dev/dxg --ipc=host
--shm-size=8G --cap-add=SYS_PTRACE --security-opt seccomp=unconfined -v
/usr/lib/wsl/lib/libdxcore.so:/usr/lib/libdxcore.so -v /opt/rocm/lib/librocdxg.so:/usr/lib/librocdxg.so -v
/opt/rocm/share/rocdxg/dids.conf:/usr/share/rocdxg/dids.conf -e HSA_ENABLE_DXG_DETECTION=1
rocm/comfyui:comfyui-0.18.2.amd0_rocm7.2.0_ubuntu24.04 python -c "import torch; print('Torch:', torch.__version__);
print('HIP:', torch.version.hip); print('Available:', torch.cuda.is_available()); print('Count:',
torch.cuda.device_count()); print('GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NONE')"
AppData\Local\Temp\comfyui-diff.md:47:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:971: UserWarning:
Can't initialize amdsmi - Error code: 34
AppData\Local\Temp\comfyui-diff.md:49:Torch: 2.10.0a0+git449b176
AppData\Local\Temp\comfyui-diff.md:50:HIP: 7.2.26015
AppData\Local\Temp\comfyui-diff.md:114:PS C:\Users\hoang> wsl ls -l /opt/rocm/lib/librocdxg.so
AppData\Local\Temp\comfyui-diff.md:115:lrwxrwxrwx 1 root root 14 Aug  4 22:55 /opt/rocm/lib/librocdxg.so ->
librocdxg.so.1
AppData\Local\Temp\comfyui-diff.md:119:b6cf5b7d8494   rocm/comfyui:comfyui-0.18.2.amd0_rocm7.2.0_ubuntu24.04   "python
/workload/Co…"   8 minutes ago   Exited (1) 24 seconds ago             comfyui
AppData\Local\Temp\comfyui-diff.md:123:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:125:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:126:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:127:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:128:comfy-aimdo failed to load: libcuda.so.1: cannot open shared object file: No
such file or directory
AppData\Local\Temp\comfyui-diff.md:129:NOTE: comfy-aimdo is currently only support for Nvidia GPUs
AppData\Local\Temp\comfyui-diff.md:130:/opt/venv/lib/python3.12/site-packages/requests/__init__.py:113:
RequestsDependencyWarning: urllib3 (2.6.3) or chardet (7.4.3)/charset_normalizer (3.4.6) doesn't match a supported
version!
AppData\Local\Temp\comfyui-diff.md:133:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:140:** Python executable: /opt/venv/bin/python
AppData\Local\Temp\comfyui-diff.md:146:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:147:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:153:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:68:
FutureWarning: The pynvml package is deprecated. Please install nvidia-ml-py instead. If you did not install pynvml
directly, please report this to the maintainers of the package that installed pynvml for you.
AppData\Local\Temp\comfyui-diff.md:155:Found comfy_kitchen backend cuda: {'available': False, 'disabled': False,
'unavailable_reason': 'CUDA not available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:156:Found comfy_kitchen backend triton: {'available': False, 'disabled': True,
'unavailable_reason': 'Neither CUDA nor XPU available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:165:    total_vram = get_total_memory(get_torch_device()) / (1024 * 1024)
AppData\Local\Temp\comfyui-diff.md:167:  File "/workload/ComfyUI/comfy/model_management.py", line 207, in
get_torch_device
AppData\Local\Temp\comfyui-diff.md:168:    return torch.device(torch.cuda.current_device())
AppData\Local\Temp\comfyui-diff.md:170:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:172:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:173:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:174:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:175:comfy-aimdo failed to load: libcuda.so.1: cannot open shared object file: No
such file or directory
AppData\Local\Temp\comfyui-diff.md:176:NOTE: comfy-aimdo is currently only support for Nvidia GPUs
AppData\Local\Temp\comfyui-diff.md:177:/opt/venv/lib/python3.12/site-packages/requests/__init__.py:113:
RequestsDependencyWarning: urllib3 (2.6.3) or chardet (7.4.3)/charset_normalizer (3.4.6) doesn'tmatch a supported
version!
AppData\Local\Temp\comfyui-diff.md:180:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:187:** Python executable: /opt/venv/bin/python
AppData\Local\Temp\comfyui-diff.md:193:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:194:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:200:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:68:
FutureWarning: The pynvml package is deprecated. Please install nvidia-ml-py instead. If you did not install pynvml
directly, please report this to the maintainers of the package that installed pynvml for you.
AppData\Local\Temp\comfyui-diff.md:203:Found comfy_kitchen backend cuda: {'available': False, 'disabled': False,
'unavailable_reason': 'CUDA not available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:204:Found comfy_kitchen backend triton: {'available': False, 'disabled': True,
'unavailable_reason': 'Neither CUDA nor XPU available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:212:    total_vram = get_total_memory(get_torch_device()) / (1024 * 1024)
AppData\Local\Temp\comfyui-diff.md:214:  File "/workload/ComfyUI/comfy/model_management.py", line 207, in
get_torch_device
AppData\Local\Temp\comfyui-diff.md:215:    return torch.device(torch.cuda.current_device())
AppData\Local\Temp\comfyui-diff.md:217:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:219:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:220:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:221:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:229:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:231:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:232:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:233:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:234:comfy-aimdo failed to load: libcuda.so.1: cannot open shared object file: No
such file or directory
AppData\Local\Temp\comfyui-diff.md:235:NOTE: comfy-aimdo is currently only support for Nvidia GPUs
AppData\Local\Temp\comfyui-diff.md:236:/opt/venv/lib/python3.12/site-packages/requests/__init__.py:113:
RequestsDependencyWarning: urllib3 (2.6.3) or chardet (7.4.3)/charset_normalizer (3.4.6) doesn't match a supported
version!
AppData\Local\Temp\comfyui-diff.md:239:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:246:** Python executable: /opt/venv/bin/python
AppData\Local\Temp\comfyui-diff.md:252:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:253:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:259:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:68:
FutureWarning: The pynvml package is deprecated. Please install nvidia-ml-py instead. If you did not install pynvml
directly, please report this to the maintainers of the package that installed pynvml for you.
AppData\Local\Temp\comfyui-diff.md:262:Found comfy_kitchen backend cuda: {'available': False, 'disabled': False,
'unavailable_reason': 'CUDA not available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:263:Found comfy_kitchen backend triton: {'available': False, 'disabled': True,
'unavailable_reason': 'Neither CUDA nor XPU available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:271:    total_vram = get_total_memory(get_torch_device()) / (1024 * 1024)
AppData\Local\Temp\comfyui-diff.md:273:  File "/workload/ComfyUI/comfy/model_management.py", line 207, in
get_torch_device
AppData\Local\Temp\comfyui-diff.md:274:    return torch.device(torch.cuda.current_device())
AppData\Local\Temp\comfyui-diff.md:276:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:278:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:279:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:280:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:281:comfy-aimdo failed to load: libcuda.so.1: cannot open shared object file: No
such file or directory
AppData\Local\Temp\comfyui-diff.md:282:NOTE: comfy-aimdo is currently only support for Nvidia GPUs
AppData\Local\Temp\comfyui-diff.md:283:/opt/venv/lib/python3.12/site-packages/requests/__init__.py:113:
RequestsDependencyWarning: urllib3 (2.6.3) or chardet (7.4.3)/charset_normalizer (3.4.6) doesn't match a supported
version!
AppData\Local\Temp\comfyui-diff.md:286:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:293:** Python executable: /opt/venv/bin/python
AppData\Local\Temp\comfyui-diff.md:299:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:300:Using Python 3.12.3 environment at: /opt/venv
AppData\Local\Temp\comfyui-diff.md:306:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:68:
FutureWarning: The pynvml package is deprecated. Please install nvidia-ml-py instead. If you did not install pynvml
directly, please report this to the maintainers of the package that installed pynvml for you.
AppData\Local\Temp\comfyui-diff.md:308:Found comfy_kitchen backend triton: {'available': False, 'disabled': True,
'unavailable_reason': 'Neither CUDA nor XPU available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:309:Found comfy_kitchen backend cuda: {'available': False, 'disabled': False,
'unavailable_reason': 'CUDA not available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:318:    total_vram = get_total_memory(get_torch_device()) / (1024 * 1024)
AppData\Local\Temp\comfyui-diff.md:320:  File "/workload/ComfyUI/comfy/model_management.py", line 207, in
get_torch_device
AppData\Local\Temp\comfyui-diff.md:321:    return torch.device(torch.cuda.current_device())
AppData\Local\Temp\comfyui-diff.md:323:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:325:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:326:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:327:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:333:PATH=/opt/venv/bin:/opt/rocm-7.2.0/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:
/usr/bin:/sbin:/bin
AppData\Local\Temp\comfyui-diff.md:335:ROCM_PATH=/opt/rocm-7.2.0
AppData\Local\Temp\comfyui-diff.md:337:PYTORCH_ROCM_ARCH=gfx942;gfx950
AppData\Local\Temp\comfyui-diff.md:341:NVTE_USE_HIPBLASLT=1
AppData\Local\Temp\comfyui-diff.md:342:NVTE_FRAMEWORK=pytorch
AppData\Local\Temp\comfyui-diff.md:343:NVTE_ROCM_ARCH=gfx942;gfx950
AppData\Local\Temp\comfyui-diff.md:344:NVTE_USE_CAST_TRANSPOSE_TRITON=1
AppData\Local\Temp\comfyui-diff.md:350:NVTE_USE_ROCM=1
AppData\Local\Temp\comfyui-diff.md:354:HIP_ARCHITECTURES=gfx942,gfx950
AppData\Local\Temp\comfyui-diff.md:356:BUILD_ROCM_VERSION=7.2
AppData\Local\Temp\comfyui-diff.md:357:FBGEMM_TBE_ROCM_HIP_BACKWARD_KERNEL=1
AppData\Local\Temp\comfyui-diff.md:358:ROCM_VERSION=72000
AppData\Local\Temp\comfyui-diff.md:359:HIPBLAS_V2=1
AppData\Local\Temp\comfyui-diff.md:368:PS C:\Users\hoang> wsl docker run --rm -it --device=/dev/dxg --ipc=host
--shm-size=8G --cap-add=SYS_PTRACE --security-opt seccomp=unconfined -v
/usr/lib/wsl/lib/libdxcore.so:/usr/lib/libdxcore.so -v /opt/rocm/lib/librocdxg.so:/usr/lib/librocdxg.so -v
/opt/rocm/share/rocdxg/dids.conf:/usr/share/rocdxg/dids.conf -e HSA_ENABLE_DXG_DETECTION=1
rocm/comfyui:comfyui-0.18.2.amd0_rocm7.2.0_ubuntu24.04 bash
AppData\Local\Temp\comfyui-diff.md:369:root@590872cfe5d4:/workspace# /opt/venv/bin/python -c "import torch;
print(torch.__version__); print('HIP:', torch.version.hip); print('CUDA:', torch.version.cuda); print('available:',
torch.cuda.is_available()); print('count:', torch.cuda.device_count())"
AppData\Local\Temp\comfyui-diff.md:371:HIP: 7.2.26015
AppData\Local\Temp\comfyui-diff.md:372:CUDA: None
AppData\Local\Temp\comfyui-diff.md:374:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:971: UserWarning:
Can't initialize amdsmi - Error code: 34
AppData\Local\Temp\comfyui-diff.md:381:PATH=/opt/venv/bin:/opt/rocm-7.2.0/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:
/usr/bin:/sbin:/bin
AppData\Local\Temp\comfyui-diff.md:383:ROCM_PATH=/opt/rocm-7.2.0
AppData\Local\Temp\comfyui-diff.md:385:PYTORCH_ROCM_ARCH=gfx942;gfx950
AppData\Local\Temp\comfyui-diff.md:389:NVTE_USE_HIPBLASLT=1
AppData\Local\Temp\comfyui-diff.md:390:NVTE_FRAMEWORK=pytorch
AppData\Local\Temp\comfyui-diff.md:391:NVTE_ROCM_ARCH=gfx942;gfx950
AppData\Local\Temp\comfyui-diff.md:392:NVTE_USE_CAST_TRANSPOSE_TRITON=1
AppData\Local\Temp\comfyui-diff.md:398:NVTE_USE_ROCM=1
AppData\Local\Temp\comfyui-diff.md:402:HIP_ARCHITECTURES=gfx942,gfx950
AppData\Local\Temp\comfyui-diff.md:404:BUILD_ROCM_VERSION=7.2
AppData\Local\Temp\comfyui-diff.md:405:FBGEMM_TBE_ROCM_HIP_BACKWARD_KERNEL=1
AppData\Local\Temp\comfyui-diff.md:406:ROCM_VERSION=72000
AppData\Local\Temp\comfyui-diff.md:407:HIPBLAS_V2=1
AppData\Local\Temp\comfyui-diff.md:446:A /root/.cache/uv/simple-v21/pypi/nvidia-nvtx.rkyv
AppData\Local\Temp\comfyui-diff.md:464:A /root/.cache/uv/simple-v21/pypi/nvidia-cuda-cupti.rkyv
AppData\Local\Temp\comfyui-diff.md:468:A /root/.cache/uv/simple-v21/pypi/nvidia-cusparselt-cu13.rkyv
AppData\Local\Temp\comfyui-diff.md:469:A /root/.cache/uv/simple-v21/pypi/nvidia-nccl-cu13.rkyv
AppData\Local\Temp\comfyui-diff.md:470:A /root/.cache/uv/simple-v21/pypi/triton.rkyv
AppData\Local\Temp\comfyui-diff.md:473:A /root/.cache/uv/simple-v21/pypi/nvidia-cudnn-cu13.rkyv
AppData\Local\Temp\comfyui-diff.md:474:A /root/.cache/uv/simple-v21/pypi/nvidia-cufft.rkyv
AppData\Local\Temp\comfyui-diff.md:480:A /root/.cache/uv/simple-v21/pypi/nvidia-cublas.rkyv
AppData\Local\Temp\comfyui-diff.md:481:A /root/.cache/uv/simple-v21/pypi/nvidia-cuda-runtime.rkyv
AppData\Local\Temp\comfyui-diff.md:483:A /root/.cache/uv/simple-v21/pypi/torchvision.rkyv
AppData\Local\Temp\comfyui-diff.md:489:A /root/.cache/uv/simple-v21/pypi/nvidia-ml-py.rkyv
AppData\Local\Temp\comfyui-diff.md:494:A /root/.cache/uv/simple-v21/pypi/nvidia-cusparse.rkyv
AppData\Local\Temp\comfyui-diff.md:502:A /root/.cache/uv/simple-v21/pypi/cuda-toolkit.rkyv
AppData\Local\Temp\comfyui-diff.md:509:A /root/.cache/uv/simple-v21/pypi/cuda-bindings.rkyv
AppData\Local\Temp\comfyui-diff.md:513:A /root/.cache/uv/simple-v21/pypi/nvidia-curand.rkyv
AppData\Local\Temp\comfyui-diff.md:517:A /root/.cache/uv/simple-v21/pypi/nvidia-nvjitlink.rkyv
AppData\Local\Temp\comfyui-diff.md:518:A /root/.cache/uv/simple-v21/pypi/nvidia-nvshmem-cu13.rkyv
AppData\Local\Temp\comfyui-diff.md:527:A /root/.cache/uv/simple-v21/pypi/nvidia-cusolver.rkyv
AppData\Local\Temp\comfyui-diff.md:532:A /root/.cache/uv/simple-v21/pypi/nvidia-cuda-nvrtc.rkyv
AppData\Local\Temp\comfyui-diff.md:541:A /root/.cache/uv/simple-v21/pypi/nvidia-cufile.rkyv
AppData\Local\Temp\comfyui-diff.md:544:A /root/.cache/uv/simple-v21/pypi/torch.rkyv
AppData\Local\Temp\comfyui-diff.md:546:A /root/.cache/uv/simple-v21/pypi/cuda-pathfinder.rkyv
AppData\Local\Temp\comfyui-diff.md:562:A /root/.cache/uv/wheels-v6/pypi/nvidia-cublas
AppData\Local\Temp\comfyui-diff.md:563:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cublas/13.1.1.3-py3-none-manylinux_2_27_x86_64
AppData\Local\Temp\comfyui-diff.md:564:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cublas/13.1.1.3-py3-none-manylinux_2_27_x86_64.http
AppData\Local\Temp\comfyui-diff.md:565:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cublas/13.1.1.3-py3-none-manylinux_2_27_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:566:A /root/.cache/uv/wheels-v6/pypi/nvidia-cuda-cupti
AppData\Local\Temp\comfyui-diff.md:567:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-cupti/13.0.85-py3-none-manylinux_2_25_x86_64
AppData\Local\Temp\comfyui-diff.md:568:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-cupti/13.0.85-py3-none-manylinux_2_25_x86_64.http
AppData\Local\Temp\comfyui-diff.md:569:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-cupti/13.0.85-py3-none-manylinux_2_25_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:598:A /root/.cache/uv/wheels-v6/pypi/nvidia-cusparselt-cu13
AppData\Local\Temp\comfyui-diff.md:599:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparselt-cu13/0.8.1-py3-none-manylinux2014_x86_64
AppData\Local\Temp\comfyui-diff.md:600:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparselt-cu13/0.8.1-py3-none-manylinux2014_x86_64.http
AppData\Local\Temp\comfyui-diff.md:601:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparselt-cu13/0.8.1-py3-none-manylinux2014_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:606:A /root/.cache/uv/wheels-v6/pypi/cuda-bindings
AppData\Local\Temp\comfyui-diff.md:607:A
/root/.cache/uv/wheels-v6/pypi/cuda-bindings/13.4.3-cp312-cp312-manylinux_2_24_x86_64.manylinux_2_28_x86_64
AppData\Local\Temp\comfyui-diff.md:608:A
/root/.cache/uv/wheels-v6/pypi/cuda-bindings/13.4.3-cp312-cp312-manylinux_2_24_x86_64.manylinux_2_28_x86_64.http
AppData\Local\Temp\comfyui-diff.md:609:A
/root/.cache/uv/wheels-v6/pypi/cuda-bindings/13.4.3-cp312-cp312-manylinux_2_24_x86_64.manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:618:A /root/.cache/uv/wheels-v6/pypi/nvidia-nccl-cu13
AppData\Local\Temp\comfyui-diff.md:619:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nccl-cu13/2.30.7-py3-none-manylinux_2_18_x86_64
AppData\Local\Temp\comfyui-diff.md:620:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nccl-cu13/2.30.7-py3-none-manylinux_2_18_x86_64.http
AppData\Local\Temp\comfyui-diff.md:621:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nccl-cu13/2.30.7-py3-none-manylinux_2_18_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:638:A /root/.cache/uv/wheels-v6/pypi/nvidia-cusparse
AppData\Local\Temp\comfyui-diff.md:639:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparse/12.6.3.3-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64
AppData\Local\Temp\comfyui-diff.md:640:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparse/12.6.3.3-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.http
AppData\Local\Temp\comfyui-diff.md:641:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparse/12.6.3.3-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:646:A /root/.cache/uv/wheels-v6/pypi/nvidia-cufft
AppData\Local\Temp\comfyui-diff.md:647:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufft/12.0.0.61-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:648:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufft/12.0.0.61-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64
AppData\Local\Temp\comfyui-diff.md:649:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufft/12.0.0.61-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.http
AppData\Local\Temp\comfyui-diff.md:666:A /root/.cache/uv/wheels-v6/pypi/nvidia-cuda-runtime
AppData\Local\Temp\comfyui-diff.md:667:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-runtime/13.0.96-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64
AppData\Local\Temp\comfyui-diff.md:668:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-runtime/13.0.96-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.http
AppData\Local\Temp\comfyui-diff.md:669:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-runtime/13.0.96-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:678:A /root/.cache/uv/wheels-v6/pypi/cuda-pathfinder
AppData\Local\Temp\comfyui-diff.md:679:A /root/.cache/uv/wheels-v6/pypi/cuda-pathfinder/1.8.3-py3-none-any
AppData\Local\Temp\comfyui-diff.md:680:A /root/.cache/uv/wheels-v6/pypi/cuda-pathfinder/1.8.3-py3-none-any.http
AppData\Local\Temp\comfyui-diff.md:681:A /root/.cache/uv/wheels-v6/pypi/cuda-pathfinder/1.8.3-py3-none-any.msgpack
AppData\Local\Temp\comfyui-diff.md:686:A /root/.cache/uv/wheels-v6/pypi/nvidia-curand
AppData\Local\Temp\comfyui-diff.md:687:A
/root/.cache/uv/wheels-v6/pypi/nvidia-curand/10.4.0.35-py3-none-manylinux_2_27_x86_64
AppData\Local\Temp\comfyui-diff.md:688:A
/root/.cache/uv/wheels-v6/pypi/nvidia-curand/10.4.0.35-py3-none-manylinux_2_27_x86_64.http
AppData\Local\Temp\comfyui-diff.md:689:A
/root/.cache/uv/wheels-v6/pypi/nvidia-curand/10.4.0.35-py3-none-manylinux_2_27_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:694:A /root/.cache/uv/wheels-v6/pypi/torchvision
AppData\Local\Temp\comfyui-diff.md:695:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.26.0-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:696:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.27.0-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:697:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.27.1-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:698:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.28.0-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:699:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.29.0-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:700:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.29.1-cp312-cp312-manylinux_2_28_x86_64
AppData\Local\Temp\comfyui-diff.md:701:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.29.1-cp312-cp312-manylinux_2_28_x86_64.http
AppData\Local\Temp\comfyui-diff.md:702:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.29.1-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:711:A /root/.cache/uv/wheels-v6/pypi/nvidia-cusolver
AppData\Local\Temp\comfyui-diff.md:712:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusolver/12.0.4.66-py3-none-manylinux_2_27_x86_64
AppData\Local\Temp\comfyui-diff.md:713:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusolver/12.0.4.66-py3-none-manylinux_2_27_x86_64.http
AppData\Local\Temp\comfyui-diff.md:714:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusolver/12.0.4.66-py3-none-manylinux_2_27_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:727:A /root/.cache/uv/wheels-v6/pypi/cuda-toolkit
AppData\Local\Temp\comfyui-diff.md:728:A /root/.cache/uv/wheels-v6/pypi/cuda-toolkit/13.0.3.0-py2.py3-none-any.msgpack
AppData\Local\Temp\comfyui-diff.md:729:A /root/.cache/uv/wheels-v6/pypi/cuda-toolkit/13.0.3.0-py2.py3-none-any
AppData\Local\Temp\comfyui-diff.md:730:A /root/.cache/uv/wheels-v6/pypi/cuda-toolkit/13.0.3.0-py2.py3-none-any.http
AppData\Local\Temp\comfyui-diff.md:731:A /root/.cache/uv/wheels-v6/pypi/nvidia-cudnn-cu13
AppData\Local\Temp\comfyui-diff.md:732:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cudnn-cu13/9.24.0.43-py3-none-manylinux_2_27_x86_64
AppData\Local\Temp\comfyui-diff.md:733:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cudnn-cu13/9.24.0.43-py3-none-manylinux_2_27_x86_64.http
AppData\Local\Temp\comfyui-diff.md:734:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cudnn-cu13/9.24.0.43-py3-none-manylinux_2_27_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:735:A /root/.cache/uv/wheels-v6/pypi/nvidia-nvtx
AppData\Local\Temp\comfyui-diff.md:736:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvtx/13.0.85-py3-none-manylinux1_x86_64.manylinux_2_5_x86_64
AppData\Local\Temp\comfyui-diff.md:737:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvtx/13.0.85-py3-none-manylinux1_x86_64.manylinux_2_5_x86_64.http
AppData\Local\Temp\comfyui-diff.md:738:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvtx/13.0.85-py3-none-manylinux1_x86_64.manylinux_2_5_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:747:A /root/.cache/uv/wheels-v6/pypi/nvidia-cuda-nvrtc
AppData\Local\Temp\comfyui-diff.md:748:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-nvrtc/13.0.88-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:749:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-nvrtc/13.0.88-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64
AppData\Local\Temp\comfyui-diff.md:750:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-nvrtc/13.0.88-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64.http
AppData\Local\Temp\comfyui-diff.md:763:A /root/.cache/uv/wheels-v6/pypi/torch
AppData\Local\Temp\comfyui-diff.md:764:A
/root/.cache/uv/wheels-v6/pypi/torch/2.14.1-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:765:A /root/.cache/uv/wheels-v6/pypi/torch/2.14.1-cp312-cp312-manylinux_2_28_x86_64
AppData\Local\Temp\comfyui-diff.md:766:A
/root/.cache/uv/wheels-v6/pypi/torch/2.14.1-cp312-cp312-manylinux_2_28_x86_64.http
AppData\Local\Temp\comfyui-diff.md:771:A /root/.cache/uv/wheels-v6/pypi/nvidia-nvshmem-cu13
AppData\Local\Temp\comfyui-diff.md:772:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvshmem-cu13/3.4.5-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.http
AppData\Local\Temp\comfyui-diff.md:773:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvshmem-cu13/3.4.5-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:774:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvshmem-cu13/3.4.5-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64
AppData\Local\Temp\comfyui-diff.md:783:A /root/.cache/uv/wheels-v6/pypi/nvidia-cufile
AppData\Local\Temp\comfyui-diff.md:784:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufile/1.15.1.6-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64
AppData\Local\Temp\comfyui-diff.md:785:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufile/1.15.1.6-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.http
AppData\Local\Temp\comfyui-diff.md:786:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufile/1.15.1.6-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:787:A /root/.cache/uv/wheels-v6/pypi/nvidia-nvjitlink
AppData\Local\Temp\comfyui-diff.md:788:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvjitlink/13.4.92-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64
AppData\Local\Temp\comfyui-diff.md:789:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvjitlink/13.4.92-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64.http
AppData\Local\Temp\comfyui-diff.md:790:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvjitlink/13.4.92-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:795:A /root/.cache/uv/wheels-v6/pypi/triton
AppData\Local\Temp\comfyui-diff.md:796:A
/root/.cache/uv/wheels-v6/pypi/triton/3.8.0-cp312-cp312-manylinux_2_27_x86_64.manylinux_2_28_x86_64
AppData\Local\Temp\comfyui-diff.md:797:A
/root/.cache/uv/wheels-v6/pypi/triton/3.8.0-cp312-cp312-manylinux_2_27_x86_64.manylinux_2_28_x86_64.http
AppData\Local\Temp\comfyui-diff.md:798:A
/root/.cache/uv/wheels-v6/pypi/triton/3.8.0-cp312-cp312-manylinux_2_27_x86_64.manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:1138:A /root/.cache/uv/archive-v0/9Y57UbtIjSZWpSdr0Gg6z/timm/utils/cuda.py
AppData\Local\Temp\comfyui-diff.md:1173:A /root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia
AppData\Local\Temp\comfyui-diff.md:1174:A /root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn
AppData\Local\Temp\comfyui-diff.md:1175:A /root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include
AppData\Local\Temp\comfyui-diff.md:1176:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_adv_v9.h
AppData\Local\Temp\comfyui-diff.md:1177:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_ops_v9.h
AppData\Local\Temp\comfyui-diff.md:1178:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_v9.h
AppData\Local\Temp\comfyui-diff.md:1179:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_version.h
AppData\Local\Temp\comfyui-diff.md:1180:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_backend.h
AppData\Local\Temp\comfyui-diff.md:1181:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_backend_v9.h
AppData\Local\Temp\comfyui-diff.md:1182:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_cnn.h
AppData\Local\Temp\comfyui-diff.md:1183:A /root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn.h
AppData\Local\Temp\comfyui-diff.md:1184:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_adv.h
AppData\Local\Temp\comfyui-diff.md:1185:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_cnn_v9.h
AppData\Local\Temp\comfyui-diff.md:1186:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_graph.h
AppData\Local\Temp\comfyui-diff.md:1187:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_graph_v9.h
AppData\Local\Temp\comfyui-diff.md:1188:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_version_v9.h
AppData\Local\Temp\comfyui-diff.md:1189:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_ops.h
AppData\Local\Temp\comfyui-diff.md:1190:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_subquadratic_ops.h
AppData\Local\Temp\comfyui-diff.md:1191:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_subquadratic_ops_v9.h
AppData\Local\Temp\comfyui-diff.md:1192:A /root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib
AppData\Local\Temp\comfyui-diff.md:1193:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_cnn.so.9
AppData\Local\Temp\comfyui-diff.md:1194:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_engines_precompiled.so.9
AppData\Local\Temp\comfyui-diff.md:1195:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_engines_runtime_compiled.so.9
AppData\Local\Temp\comfyui-diff.md:1196:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_engines_tensor_ir.so.9
AppData\Local\Temp\comfyui-diff.md:1197:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_graph.so.9
AppData\Local\Temp\comfyui-diff.md:1198:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_heuristic.so.9
AppData\Local\Temp\comfyui-diff.md:1199:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_ops.so.9
AppData\Local\Temp\comfyui-diff.md:1200:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn.so.9
AppData\Local\Temp\comfyui-diff.md:1201:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_adv.so.9
AppData\Local\Temp\comfyui-diff.md:1202:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_ext.so.9
AppData\Local\Temp\comfyui-diff.md:1203:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info
AppData\Local\Temp\comfyui-diff.md:1204:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/top_level.txt
AppData\Local\Temp\comfyui-diff.md:1205:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/METADATA
AppData\Local\Temp\comfyui-diff.md:1206:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/RECORD
AppData\Local\Temp\comfyui-diff.md:1207:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/WHEEL
AppData\Local\Temp\comfyui-diff.md:1208:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/licenses
AppData\Local\Temp\comfyui-diff.md:1209:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/licenses/License.txt
AppData\Local\Temp\comfyui-diff.md:1280:A /root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia
AppData\Local\Temp\comfyui-diff.md:1281:A /root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13
AppData\Local\Temp\comfyui-diff.md:1282:A /root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/include
AppData\Local\Temp\comfyui-diff.md:1283:A /root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/include/nvrtc.h
AppData\Local\Temp\comfyui-diff.md:1284:A /root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/lib
AppData\Local\Temp\comfyui-diff.md:1285:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/lib/libnvrtc.alt.so.13
AppData\Local\Temp\comfyui-diff.md:1286:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/lib/libnvrtc.so.13
AppData\Local\Temp\comfyui-diff.md:1287:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/lib/libnvrtc-builtins.alt.so.13.0
AppData\Local\Temp\comfyui-diff.md:1288:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/lib/libnvrtc-builtins.so.13.0
AppData\Local\Temp\comfyui-diff.md:1289:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info
AppData\Local\Temp\comfyui-diff.md:1290:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/licenses
AppData\Local\Temp\comfyui-diff.md:1291:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/licenses/License.txt
AppData\Local\Temp\comfyui-diff.md:1292:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/top_level.txt
AppData\Local\Temp\comfyui-diff.md:1293:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/METADATA
AppData\Local\Temp\comfyui-diff.md:1294:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/RECORD
AppData\Local\Temp\comfyui-diff.md:1295:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/WHEEL
AppData\Local\Temp\comfyui-diff.md:1405:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia
AppData\Local\Temp\comfyui-diff.md:1406:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13
AppData\Local\Temp\comfyui-diff.md:1407:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include
AppData\Local\Temp\comfyui-diff.md:1408:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/cublasXt.h
AppData\Local\Temp\comfyui-diff.md:1409:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/cublas_api.h
AppData\Local\Temp\comfyui-diff.md:1410:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/cublas_v2.h
AppData\Local\Temp\comfyui-diff.md:1411:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/nvblas.h
AppData\Local\Temp\comfyui-diff.md:1412:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/cublas.h
AppData\Local\Temp\comfyui-diff.md:1413:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/cublasLt.h
AppData\Local\Temp\comfyui-diff.md:1414:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/lib
AppData\Local\Temp\comfyui-diff.md:1415:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/lib/libcublasLt.so.13
AppData\Local\Temp\comfyui-diff.md:1416:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/lib/libnvblas.so.13
AppData\Local\Temp\comfyui-diff.md:1417:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/lib/libcublas.so.13
AppData\Local\Temp\comfyui-diff.md:1418:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info
AppData\Local\Temp\comfyui-diff.md:1419:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/licenses
AppData\Local\Temp\comfyui-diff.md:1420:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/licenses/License.txt
AppData\Local\Temp\comfyui-diff.md:1421:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/top_level.txt
AppData\Local\Temp\comfyui-diff.md:1422:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/METADATA
AppData\Local\Temp\comfyui-diff.md:1423:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/RECORD
AppData\Local\Temp\comfyui-diff.md:1424:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/WHEEL
AppData\Local\Temp\comfyui-diff.md:1867:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda
AppData\Local\Temp\comfyui-diff.md:1868:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/cpp_function_wrappers.cu
AppData\Local\Temp\comfyui-diff.md:1869:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator
AppData\Local\Temp\comfyui-diff.md:1870:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/kernelapi.py
AppData\Local\Temp\comfyui-diff.md:1871:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/reduction.py
AppData\Local\Temp\comfyui-diff.md:1872:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/vector_types.py
AppData\Local\Temp\comfyui-diff.md:1873:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/__init__.py
AppData\Local\Temp\comfyui-diff.md:1874:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/api.py
AppData\Local\Temp\comfyui-diff.md:1875:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/compiler.py
AppData\Local\Temp\comfyui-diff.md:1876:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv
AppData\Local\Temp\comfyui-diff.md:1877:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/devices.py
AppData\Local\Temp\comfyui-diff.md:1878:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/driver.py
AppData\Local\Temp\comfyui-diff.md:1879:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/drvapi.py
AppData\Local\Temp\comfyui-diff.md:1880:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/dummyarray.py
AppData\Local\Temp\comfyui-diff.md:1881:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/error.py
AppData\Local\Temp\comfyui-diff.md:1882:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/libs.py
AppData\Local\Temp\comfyui-diff.md:1883:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/nvvm.py
AppData\Local\Temp\comfyui-diff.md:1884:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/runtime.py
AppData\Local\Temp\comfyui-diff.md:1885:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/__init__.py
AppData\Local\Temp\comfyui-diff.md:1886:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/devicearray.py
AppData\Local\Temp\comfyui-diff.md:1887:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/kernel.py
AppData\Local\Temp\comfyui-diff.md:1888:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/target.py
AppData\Local\Temp\comfyui-diff.md:1889:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/api.py
AppData\Local\Temp\comfyui-diff.md:1890:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/api_util.py
AppData\Local\Temp\comfyui-diff.md:1891:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/nvvmutils.py
AppData\Local\Temp\comfyui-diff.md:1892:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests
AppData\Local\Temp\comfyui-diff.md:1893:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/nocuda
AppData\Local\Temp\comfyui-diff.md:1894:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/nocuda/__init__.py
AppData\Local\Temp\comfyui-diff.md:1895:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/nocuda/test_dummyarray.py
AppData\Local\Temp\comfyui-diff.md:1896:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/nocuda/test_function_resolution.py
AppData\Local\Temp\comfyui-diff.md:1897:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/nocuda/test_import.py
AppData\Local\Temp\comfyui-diff.md:1898:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/nocuda/test_library_lookup.py
AppData\Local\Temp\comfyui-diff.md:1899:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/nocuda/test_nvvm.py
AppData\Local\Temp\comfyui-diff.md:1900:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/__init__.py
AppData\Local\Temp\comfyui-diff.md:1901:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv
AppData\Local\Temp\comfyui-diff.md:1902:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/__init__.py
AppData\Local\Temp\comfyui-diff.md:1903:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_array_attr.py
AppData\Local\Temp\comfyui-diff.md:1904:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_cuda_ndarray.py
AppData\Local\Temp\comfyui-diff.md:1905:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_events.py
AppData\Local\Temp\comfyui-diff.md:1906:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_linker.py
AppData\Local\Temp\comfyui-diff.md:1907:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_deallocations.py
AppData\Local\Temp\comfyui-diff.md:1908:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_reset_device.py
AppData\Local\Temp\comfyui-diff.md:1909:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_runtime.py
AppData\Local\Temp\comfyui-diff.md:1910:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_nvvm_driver.py
AppData\Local\Temp\comfyui-diff.md:1911:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_cuda_array_slicing.py
AppData\Local\Temp\comfyui-diff.md:1912:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_cuda_auto_context.py
AppData\Local\Temp\comfyui-diff.md:1913:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_cuda_memory.py
AppData\Local\Temp\comfyui-diff.md:1914:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_managed_alloc.py
AppData\Local\Temp\comfyui-diff.md:1915:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_select_device.py
AppData\Local\Temp\comfyui-diff.md:1916:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_cuda_libraries.py
AppData\Local\Temp\comfyui-diff.md:1917:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_host_alloc.py
AppData\Local\Temp\comfyui-diff.md:1918:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_inline_ptx.py
AppData\Local\Temp\comfyui-diff.md:1919:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_mvc.py
AppData\Local\Temp\comfyui-diff.md:1920:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_profiler.py
AppData\Local\Temp\comfyui-diff.md:1921:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_detect.py
AppData\Local\Temp\comfyui-diff.md:1922:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_ptds.py
AppData\Local\Temp\comfyui-diff.md:1923:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_cuda_devicerecord.py
AppData\Local\Temp\comfyui-diff.md:1924:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_cuda_driver.py
AppData\Local\Temp\comfyui-diff.md:1925:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_emm_plugins.py
AppData\Local\Temp\comfyui-diff.md:1926:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_is_fp16.py
AppData\Local\Temp\comfyui-diff.md:1927:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_streams.py
AppData\Local\Temp\comfyui-diff.md:1928:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_context_stack.py
AppData\Local\Temp\comfyui-diff.md:1929:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_init.py
AppData\Local\Temp\comfyui-diff.md:1930:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudadrv/test_pinned.py
AppData\Local\Temp\comfyui-diff.md:1931:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy
AppData\Local\Temp\comfyui-diff.md:1932:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_vectorize_complex.py
AppData\Local\Temp\comfyui-diff.md:1933:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_idiv.py
AppData\Local\Temp\comfyui-diff.md:1934:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/cache_usecases.py
AppData\Local\Temp\comfyui-diff.md:1935:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_atomics.py
AppData\Local\Temp\comfyui-diff.md:1936:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_extending.py
AppData\Local\Temp\comfyui-diff.md:1937:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_retrieve_autoconverted_arrays.py
AppData\Local\Temp\comfyui-diff.md:1938:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_warning.py
AppData\Local\Temp\comfyui-diff.md:1939:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_alignment.py
AppData\Local\Temp\comfyui-diff.md:1940:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_dispatcher.py
AppData\Local\Temp\comfyui-diff.md:1941:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_inspect.py
AppData\Local\Temp\comfyui-diff.md:1942:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_sm_creation.py
AppData\Local\Temp\comfyui-diff.md:1943:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/cache_with_cpu_usecases.py
AppData\Local\Temp\comfyui-diff.md:1944:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/extensions_usecases.py
AppData\Local\Temp\comfyui-diff.md:1945:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_array_methods.py
AppData\Local\Temp\comfyui-diff.md:1946:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_compiler.py
AppData\Local\Temp\comfyui-diff.md:1947:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_datetime.py
AppData\Local\Temp\comfyui-diff.md:1948:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_gufunc_scalar.py
AppData\Local\Temp\comfyui-diff.md:1949:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_optimization.py
AppData\Local\Temp\comfyui-diff.md:1950:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/__init__.py
AppData\Local\Temp\comfyui-diff.md:1951:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_forall.py
AppData\Local\Temp\comfyui-diff.md:1952:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_intrinsics.py
AppData\Local\Temp\comfyui-diff.md:1953:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_math.py
AppData\Local\Temp\comfyui-diff.md:1954:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_userexc.py
AppData\Local\Temp\comfyui-diff.md:1955:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_vectorize_device.py
AppData\Local\Temp\comfyui-diff.md:1956:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_cooperative_groups.py
AppData\Local\Temp\comfyui-diff.md:1957:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_exception.py
AppData\Local\Temp\comfyui-diff.md:1958:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_freevar.py
AppData\Local\Temp\comfyui-diff.md:1959:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_lineinfo.py
AppData\Local\Temp\comfyui-diff.md:1960:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_mandel.py
AppData\Local\Temp\comfyui-diff.md:1961:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_cuda_jit_no_types.py
AppData\Local\Temp\comfyui-diff.md:1962:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_frexp_ldexp.py
AppData\Local\Temp\comfyui-diff.md:1963:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_libdevice.py
AppData\Local\Temp\comfyui-diff.md:1964:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_localmem.py
AppData\Local\Temp\comfyui-diff.md:1965:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_overload.py
AppData\Local\Temp\comfyui-diff.md:1966:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_vectorize_decor.py
AppData\Local\Temp\comfyui-diff.md:1967:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_casting.py
AppData\Local\Temp\comfyui-diff.md:1968:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_blackscholes.py
AppData\Local\Temp\comfyui-diff.md:1969:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_constmem.py
AppData\Local\Temp\comfyui-diff.md:1970:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_enums.py
AppData\Local\Temp\comfyui-diff.md:1971:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_montecarlo.py
AppData\Local\Temp\comfyui-diff.md:1972:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_nondet.py
AppData\Local\Temp\comfyui-diff.md:1973:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_reduction.py
AppData\Local\Temp\comfyui-diff.md:1974:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_serialize.py
AppData\Local\Temp\comfyui-diff.md:1975:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_caching.py
AppData\Local\Temp\comfyui-diff.md:1976:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_const_string.py
AppData\Local\Temp\comfyui-diff.md:1977:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_debuginfo.py
AppData\Local\Temp\comfyui-diff.md:1978:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_ipc.py
AppData\Local\Temp\comfyui-diff.md:1979:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_laplace.py
AppData\Local\Temp\comfyui-diff.md:1980:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_slicing.py
AppData\Local\Temp\comfyui-diff.md:1981:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_warp_ops.py
AppData\Local\Temp\comfyui-diff.md:1982:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_lang.py
AppData\Local\Temp\comfyui-diff.md:1983:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/recursion_usecases.py
AppData\Local\Temp\comfyui-diff.md:1984:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_array_args.py
AppData\Local\Temp\comfyui-diff.md:1985:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_cffi.py
AppData\Local\Temp\comfyui-diff.md:1986:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_complex.py
AppData\Local\Temp\comfyui-diff.md:1987:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_debug.py
AppData\Local\Temp\comfyui-diff.md:1988:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_sm.py
AppData\Local\Temp\comfyui-diff.md:1989:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_sync.py
AppData\Local\Temp\comfyui-diff.md:1990:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_complex_kernel.py
AppData\Local\Temp\comfyui-diff.md:1991:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_device_func.py
AppData\Local\Temp\comfyui-diff.md:1992:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_random.py
AppData\Local\Temp\comfyui-diff.md:1993:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_transpose.py
AppData\Local\Temp\comfyui-diff.md:1994:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_vector_type.py
AppData\Local\Temp\comfyui-diff.md:1995:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_vectorize_scalar_arg.py
AppData\Local\Temp\comfyui-diff.md:1996:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_globals.py
AppData\Local\Temp\comfyui-diff.md:1997:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_multiprocessing.py
AppData\Local\Temp\comfyui-diff.md:1998:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_powi.py
AppData\Local\Temp\comfyui-diff.md:1999:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_py2_div_issue.py
AppData\Local\Temp\comfyui-diff.md:2000:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_record_dtype.py
AppData\Local\Temp\comfyui-diff.md:2001:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_boolean.py
AppData\Local\Temp\comfyui-diff.md:2002:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_multithreads.py
AppData\Local\Temp\comfyui-diff.md:2003:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_operator.py
AppData\Local\Temp\comfyui-diff.md:2004:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_gufunc_scheduling.py
AppData\Local\Temp\comfyui-diff.md:2005:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_matmul.py
AppData\Local\Temp\comfyui-diff.md:2006:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_multigpu.py
AppData\Local\Temp\comfyui-diff.md:2007:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_vectorize.py
AppData\Local\Temp\comfyui-diff.md:2008:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_array.py
AppData\Local\Temp\comfyui-diff.md:2009:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_errors.py
AppData\Local\Temp\comfyui-diff.md:2010:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_fastmath.py
AppData\Local\Temp\comfyui-diff.md:2011:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_minmax.py
AppData\Local\Temp\comfyui-diff.md:2012:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_print.py
AppData\Local\Temp\comfyui-diff.md:2013:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_recursion.py
AppData\Local\Temp\comfyui-diff.md:2014:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_ufuncs.py
AppData\Local\Temp\comfyui-diff.md:2015:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_cuda_array_interface.py
AppData\Local\Temp\comfyui-diff.md:2016:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_iterators.py
AppData\Local\Temp\comfyui-diff.md:2017:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudapy/test_gufunc.py
AppData\Local\Temp\comfyui-diff.md:2018:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudasim
AppData\Local\Temp\comfyui-diff.md:2019:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudasim/__init__.py
AppData\Local\Temp\comfyui-diff.md:2020:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudasim/support.py
AppData\Local\Temp\comfyui-diff.md:2021:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/cudasim/test_cudasim_issues.py
AppData\Local\Temp\comfyui-diff.md:2022:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/data
AppData\Local\Temp\comfyui-diff.md:2023:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/data/error.cu
AppData\Local\Temp\comfyui-diff.md:2024:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/data/jitlink.cu
AppData\Local\Temp\comfyui-diff.md:2025:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/data/jitlink.ptx
AppData\Local\Temp\comfyui-diff.md:2026:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/data/warn.cu
AppData\Local\Temp\comfyui-diff.md:2027:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/data/__init__.py
AppData\Local\Temp\comfyui-diff.md:2028:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/data/cuda_include.cu
AppData\Local\Temp\comfyui-diff.md:2029:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples
AppData\Local\Temp\comfyui-diff.md:2030:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/test_ffi.py
AppData\Local\Temp\comfyui-diff.md:2031:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/test_laplace.py
AppData\Local\Temp\comfyui-diff.md:2032:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/test_matmul.py
AppData\Local\Temp\comfyui-diff.md:2033:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/test_random.py
AppData\Local\Temp\comfyui-diff.md:2034:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/test_reduction.py
AppData\Local\Temp\comfyui-diff.md:2035:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/test_sessionize.py
AppData\Local\Temp\comfyui-diff.md:2036:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/__init__.py
AppData\Local\Temp\comfyui-diff.md:2037:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/test_cpu_gpu_compat.py
AppData\Local\Temp\comfyui-diff.md:2038:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/test_montecarlo.py
AppData\Local\Temp\comfyui-diff.md:2039:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/test_ufunc.py
AppData\Local\Temp\comfyui-diff.md:2040:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/test_vecadd.py
AppData\Local\Temp\comfyui-diff.md:2041:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/ffi
AppData\Local\Temp\comfyui-diff.md:2042:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/ffi/__init__.py
AppData\Local\Temp\comfyui-diff.md:2043:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/ffi/functions.cu
AppData\Local\Temp\comfyui-diff.md:2044:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/doc_examples/test_cg.py
AppData\Local\Temp\comfyui-diff.md:2045:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/ufuncs.py
AppData\Local\Temp\comfyui-diff.md:2046:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/args.py
AppData\Local\Temp\comfyui-diff.md:2047:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/cudaimpl.py
AppData\Local\Temp\comfyui-diff.md:2048:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/deviceufunc.py
AppData\Local\Temp\comfyui-diff.md:2049:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/mathimpl.py
AppData\Local\Temp\comfyui-diff.md:2050:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/models.py
AppData\Local\Temp\comfyui-diff.md:2051:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/vector_types.py
AppData\Local\Temp\comfyui-diff.md:2052:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/__init__.py
AppData\Local\Temp\comfyui-diff.md:2053:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/decorators.py
AppData\Local\Temp\comfyui-diff.md:2054:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/dispatcher.py
AppData\Local\Temp\comfyui-diff.md:2055:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/initialize.py
AppData\Local\Temp\comfyui-diff.md:2056:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/random.py
AppData\Local\Temp\comfyui-diff.md:2057:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/descriptor.py
AppData\Local\Temp\comfyui-diff.md:2058:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/extending.py
AppData\Local\Temp\comfyui-diff.md:2059:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/intrinsics.py
AppData\Local\Temp\comfyui-diff.md:2060:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/libdevice.py
AppData\Local\Temp\comfyui-diff.md:2061:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/libdevicedecl.py
AppData\Local\Temp\comfyui-diff.md:2062:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/printimpl.py
AppData\Local\Temp\comfyui-diff.md:2063:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/cg.py
AppData\Local\Temp\comfyui-diff.md:2064:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/device_init.py
AppData\Local\Temp\comfyui-diff.md:2065:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/intrinsic_wrapper.py
AppData\Local\Temp\comfyui-diff.md:2066:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator_init.py


PS C:\Users\hoang> Select-String `
>>   -Path "$env:TEMP\comfyui-diff.md" `
>>   -Pattern '^C /opt/venv|^A /opt/venv|^D /opt/venv' `
>>   -CaseSensitive:$false |
>>   Select-Object -First 300

AppData\Local\Temp\comfyui-diff.md:25635:C /opt/venv
AppData\Local\Temp\comfyui-diff.md:25636:C /opt/venv/lib
AppData\Local\Temp\comfyui-diff.md:25637:C /opt/venv/lib/python3.12
AppData\Local\Temp\comfyui-diff.md:25638:C /opt/venv/lib/python3.12/site-packages
AppData\Local\Temp\comfyui-diff.md:25639:A /opt/venv/lib/python3.12/site-packages/nvidia_cublas-13.1.1.3.dist-info
AppData\Local\Temp\comfyui-diff.md:25640:A
/opt/venv/lib/python3.12/site-packages/nvidia_cublas-13.1.1.3.dist-info/METADATA
AppData\Local\Temp\comfyui-diff.md:25641:A
/opt/venv/lib/python3.12/site-packages/nvidia_cublas-13.1.1.3.dist-info/RECORD
AppData\Local\Temp\comfyui-diff.md:25642:A
/opt/venv/lib/python3.12/site-packages/nvidia_cublas-13.1.1.3.dist-info/REQUESTED
AppData\Local\Temp\comfyui-diff.md:25643:A
/opt/venv/lib/python3.12/site-packages/nvidia_cublas-13.1.1.3.dist-info/WHEEL
AppData\Local\Temp\comfyui-diff.md:25644:A
/opt/venv/lib/python3.12/site-packages/nvidia_cublas-13.1.1.3.dist-info/licenses
AppData\Local\Temp\comfyui-diff.md:25645:A
/opt/venv/lib/python3.12/site-packages/nvidia_cublas-13.1.1.3.dist-info/licenses/License.txt
AppData\Local\Temp\comfyui-diff.md:25646:A
/opt/venv/lib/python3.12/site-packages/nvidia_cublas-13.1.1.3.dist-info/top_level.txt
AppData\Local\Temp\comfyui-diff.md:25647:A
/opt/venv/lib/python3.12/site-packages/nvidia_cublas-13.1.1.3.dist-info/INSTALLER
AppData\Local\Temp\comfyui-diff.md:25648:A
/opt/venv/lib/python3.12/site-packages/simsimd.cpython-312-x86_64-linux-gnu.so
AppData\Local\Temp\comfyui-diff.md:25649:A /opt/venv/lib/python3.12/site-packages/albumentations-2.0.8.dist-info
AppData\Local\Temp\comfyui-diff.md:25650:A /opt/venv/lib/python3.12/site-packages/albumentations-2.0.8.dist-info/WHEEL
AppData\Local\Temp\comfyui-diff.md:25651:A
/opt/venv/lib/python3.12/site-packages/albumentations-2.0.8.dist-info/licenses
AppData\Local\Temp\comfyui-diff.md:25652:A
/opt/venv/lib/python3.12/site-packages/albumentations-2.0.8.dist-info/licenses/LICENSE
AppData\Local\Temp\comfyui-diff.md:25653:A
/opt/venv/lib/python3.12/site-packages/albumentations-2.0.8.dist-info/top_level.txt
AppData\Local\Temp\comfyui-diff.md:25654:A
/opt/venv/lib/python3.12/site-packages/albumentations-2.0.8.dist-info/INSTALLER
AppData\Local\Temp\comfyui-diff.md:25655:A
/opt/venv/lib/python3.12/site-packages/albumentations-2.0.8.dist-info/METADATA
AppData\Local\Temp\comfyui-diff.md:25656:A /opt/venv/lib/python3.12/site-packages/albumentations-2.0.8.dist-info/RECORD
AppData\Local\Temp\comfyui-diff.md:25657:A
/opt/venv/lib/python3.12/site-packages/albumentations-2.0.8.dist-info/REQUESTED
AppData\Local\Temp\comfyui-diff.md:25658:A /opt/venv/lib/python3.12/site-packages/nvidia_nvtx-13.0.85.dist-info
AppData\Local\Temp\comfyui-diff.md:25659:A
/opt/venv/lib/python3.12/site-packages/nvidia_nvtx-13.0.85.dist-info/INSTALLER
AppData\Local\Temp\comfyui-diff.md:25660:A
/opt/venv/lib/python3.12/site-packages/nvidia_nvtx-13.0.85.dist-info/METADATA
AppData\Local\Temp\comfyui-diff.md:25661:A /opt/venv/lib/python3.12/site-packages/nvidia_nvtx-13.0.85.dist-info/RECORD
AppData\Local\Temp\comfyui-diff.md:25662:A
/opt/venv/lib/python3.12/site-packages/nvidia_nvtx-13.0.85.dist-info/REQUESTED
AppData\Local\Temp\comfyui-diff.md:25663:A /opt/venv/lib/python3.12/site-packages/nvidia_nvtx-13.0.85.dist-info/WHEEL
AppData\Local\Temp\comfyui-diff.md:25664:A
/opt/venv/lib/python3.12/site-packages/nvidia_nvtx-13.0.85.dist-info/licenses
AppData\Local\Temp\comfyui-diff.md:25665:A
/opt/venv/lib/python3.12/site-packages/nvidia_nvtx-13.0.85.dist-info/licenses/License.txt
AppData\Local\Temp\comfyui-diff.md:25666:A
/opt/venv/lib/python3.12/site-packages/nvidia_nvtx-13.0.85.dist-info/top_level.txt
AppData\Local\Temp\comfyui-diff.md:25667:A /opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs
AppData\Local\Temp\comfyui-diff.md:25668:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libaom-a0d22147.so.3.14.1
AppData\Local\Temp\comfyui-diff.md:25669:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libgfortran-83c28eba.so.5.0.0
AppData\Local\Temp\comfyui-diff.md:25670:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libswscale-fe215b0b.so.9.5.101
AppData\Local\Temp\comfyui-diff.md:25671:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libavcodec-c4204469.so.62.28.101
AppData\Local\Temp\comfyui-diff.md:25672:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libavformat-4762a711.so.62.12.101
AppData\Local\Temp\comfyui-diff.md:25673:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libavif-43e630fc.so.16.4.2
AppData\Local\Temp\comfyui-diff.md:25674:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libssl-81259c47.so.1.1.1k
AppData\Local\Temp\comfyui-diff.md:25675:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libvpx-4fb239ff.so.11.0.1
AppData\Local\Temp\comfyui-diff.md:25676:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libavutil-befbbc48.so.60.26.101
AppData\Local\Temp\comfyui-diff.md:25677:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libcrypto-5409cd36.so.1.1.1k
AppData\Local\Temp\comfyui-diff.md:25678:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libdrm-b0291a67.so.2.4.0
AppData\Local\Temp\comfyui-diff.md:25679:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libopenblasp-r0-59ffcd50.3.15.so
AppData\Local\Temp\comfyui-diff.md:25680:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libpng16-529cb57a.so.16.58.0
AppData\Local\Temp\comfyui-diff.md:25681:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libquadmath-2284e583.so.0.0.0
AppData\Local\Temp\comfyui-diff.md:25682:A
/opt/venv/lib/python3.12/site-packages/opencv_python_headless.libs/libswresample-2dfded3b.so.6.3.101
AppData\Local\Temp\comfyui-diff.md:25683:A /opt/venv/lib/python3.12/site-packages/timm
AppData\Local\Temp\comfyui-diff.md:25684:A /opt/venv/lib/python3.12/site-packages/timm/utils
AppData\Local\Temp\comfyui-diff.md:25685:A /opt/venv/lib/python3.12/site-packages/timm/utils/onnx.py
AppData\Local\Temp\comfyui-diff.md:25686:A /opt/venv/lib/python3.12/site-packages/timm/utils/random.py
AppData\Local\Temp\comfyui-diff.md:25687:A /opt/venv/lib/python3.12/site-packages/timm/utils/attention_extract.py
AppData\Local\Temp\comfyui-diff.md:25688:A /opt/venv/lib/python3.12/site-packages/timm/utils/checkpoint_saver.py
AppData\Local\Temp\comfyui-diff.md:25689:A /opt/venv/lib/python3.12/site-packages/timm/utils/distributed.py
AppData\Local\Temp\comfyui-diff.md:25690:A /opt/venv/lib/python3.12/site-packages/timm/utils/agc.py
AppData\Local\Temp\comfyui-diff.md:25691:A /opt/venv/lib/python3.12/site-packages/timm/utils/log.py
AppData\Local\Temp\comfyui-diff.md:25692:A /opt/venv/lib/python3.12/site-packages/timm/utils/metrics.py
AppData\Local\Temp\comfyui-diff.md:25693:A /opt/venv/lib/python3.12/site-packages/timm/utils/model_ema.py
AppData\Local\Temp\comfyui-diff.md:25694:A /opt/venv/lib/python3.12/site-packages/timm/utils/jit.py
AppData\Local\Temp\comfyui-diff.md:25695:A /opt/venv/lib/python3.12/site-packages/timm/utils/misc.py
AppData\Local\Temp\comfyui-diff.md:25696:A /opt/venv/lib/python3.12/site-packages/timm/utils/model.py
AppData\Local\Temp\comfyui-diff.md:25697:A /opt/venv/lib/python3.12/site-packages/timm/utils/summary.py
AppData\Local\Temp\comfyui-diff.md:25698:A /opt/venv/lib/python3.12/site-packages/timm/utils/__init__.py
AppData\Local\Temp\comfyui-diff.md:25699:A /opt/venv/lib/python3.12/site-packages/timm/utils/clip_grad.py
AppData\Local\Temp\comfyui-diff.md:25700:A /opt/venv/lib/python3.12/site-packages/timm/utils/cuda.py
AppData\Local\Temp\comfyui-diff.md:25701:A /opt/venv/lib/python3.12/site-packages/timm/utils/decay_batch.py
AppData\Local\Temp\comfyui-diff.md:25702:A /opt/venv/lib/python3.12/site-packages/timm/version.py
AppData\Local\Temp\comfyui-diff.md:25703:A /opt/venv/lib/python3.12/site-packages/timm/__init__.py
AppData\Local\Temp\comfyui-diff.md:25704:A /opt/venv/lib/python3.12/site-packages/timm/data
AppData\Local\Temp\comfyui-diff.md:25705:A /opt/venv/lib/python3.12/site-packages/timm/data/__init__.py
AppData\Local\Temp\comfyui-diff.md:25706:A /opt/venv/lib/python3.12/site-packages/timm/data/_info
AppData\Local\Temp\comfyui-diff.md:25707:A /opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet_synsets.txt
AppData\Local\Temp\comfyui-diff.md:25708:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet22k_ms_to_22k_indices.txt
AppData\Local\Temp\comfyui-diff.md:25709:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet22k_synsets.txt
AppData\Local\Temp\comfyui-diff.md:25710:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet_real_labels.json
AppData\Local\Temp\comfyui-diff.md:25711:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet12k_synsets.txt
AppData\Local\Temp\comfyui-diff.md:25712:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet21k_goog_synsets.txt
AppData\Local\Temp\comfyui-diff.md:25713:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet21k_goog_to_12k_indices.txt
AppData\Local\Temp\comfyui-diff.md:25714:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet21k_miil_synsets.txt
AppData\Local\Temp\comfyui-diff.md:25715:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet22k_ms_to_12k_indices.txt
AppData\Local\Temp\comfyui-diff.md:25716:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet_r_indices.txt
AppData\Local\Temp\comfyui-diff.md:25717:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/mini_imagenet_synsets.txt
AppData\Local\Temp\comfyui-diff.md:25718:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet21k_miil_w21_synsets.txt
AppData\Local\Temp\comfyui-diff.md:25719:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet_a_indices.txt
AppData\Local\Temp\comfyui-diff.md:25720:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet_r_synsets.txt
AppData\Local\Temp\comfyui-diff.md:25721:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/mini_imagenet_indices.txt
AppData\Local\Temp\comfyui-diff.md:25722:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet21k_goog_to_22k_indices.txt
AppData\Local\Temp\comfyui-diff.md:25723:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet22k_ms_synsets.txt
AppData\Local\Temp\comfyui-diff.md:25724:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet22k_to_12k_indices.txt
AppData\Local\Temp\comfyui-diff.md:25725:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet_a_synsets.txt
AppData\Local\Temp\comfyui-diff.md:25726:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet_synset_to_definition.txt
AppData\Local\Temp\comfyui-diff.md:25727:A
/opt/venv/lib/python3.12/site-packages/timm/data/_info/imagenet_synset_to_lemma.txt
AppData\Local\Temp\comfyui-diff.md:25728:A /opt/venv/lib/python3.12/site-packages/timm/data/constants.py
AppData\Local\Temp\comfyui-diff.md:25729:A /opt/venv/lib/python3.12/site-packages/timm/data/loader.py
AppData\Local\Temp\comfyui-diff.md:25730:A /opt/venv/lib/python3.12/site-packages/timm/data/naflex_loader.py
AppData\Local\Temp\comfyui-diff.md:25731:A /opt/venv/lib/python3.12/site-packages/timm/data/naflex_mixup.py
AppData\Local\Temp\comfyui-diff.md:25732:A /opt/venv/lib/python3.12/site-packages/timm/data/readers
AppData\Local\Temp\comfyui-diff.md:25733:A /opt/venv/lib/python3.12/site-packages/timm/data/readers/reader_hfds.py
AppData\Local\Temp\comfyui-diff.md:25734:A /opt/venv/lib/python3.12/site-packages/timm/data/readers/reader_hfids.py
AppData\Local\Temp\comfyui-diff.md:25735:A
/opt/venv/lib/python3.12/site-packages/timm/data/readers/reader_image_folder.py
AppData\Local\Temp\comfyui-diff.md:25736:A /opt/venv/lib/python3.12/site-packages/timm/data/readers/reader_image_tar.py
AppData\Local\Temp\comfyui-diff.md:25737:A /opt/venv/lib/python3.12/site-packages/timm/data/readers/__init__.py
AppData\Local\Temp\comfyui-diff.md:25738:A /opt/venv/lib/python3.12/site-packages/timm/data/readers/reader_factory.py
AppData\Local\Temp\comfyui-diff.md:25739:A
/opt/venv/lib/python3.12/site-packages/timm/data/readers/reader_image_in_tar.py
AppData\Local\Temp\comfyui-diff.md:25740:A /opt/venv/lib/python3.12/site-packages/timm/data/readers/reader_tfds.py
AppData\Local\Temp\comfyui-diff.md:25741:A /opt/venv/lib/python3.12/site-packages/timm/data/readers/reader_wds.py
AppData\Local\Temp\comfyui-diff.md:25742:A /opt/venv/lib/python3.12/site-packages/timm/data/readers/shared_count.py
AppData\Local\Temp\comfyui-diff.md:25743:A /opt/venv/lib/python3.12/site-packages/timm/data/readers/class_map.py
AppData\Local\Temp\comfyui-diff.md:25744:A /opt/venv/lib/python3.12/site-packages/timm/data/readers/img_extensions.py
AppData\Local\Temp\comfyui-diff.md:25745:A /opt/venv/lib/python3.12/site-packages/timm/data/readers/reader.py
AppData\Local\Temp\comfyui-diff.md:25746:A /opt/venv/lib/python3.12/site-packages/timm/data/scheduled_sampler.py
AppData\Local\Temp\comfyui-diff.md:25747:A /opt/venv/lib/python3.12/site-packages/timm/data/dataset.py
AppData\Local\Temp\comfyui-diff.md:25748:A /opt/venv/lib/python3.12/site-packages/timm/data/dataset_factory.py
AppData\Local\Temp\comfyui-diff.md:25749:A /opt/venv/lib/python3.12/site-packages/timm/data/dataset_info.py
AppData\Local\Temp\comfyui-diff.md:25750:A /opt/venv/lib/python3.12/site-packages/timm/data/imagenet_info.py
AppData\Local\Temp\comfyui-diff.md:25751:A /opt/venv/lib/python3.12/site-packages/timm/data/mixup.py
AppData\Local\Temp\comfyui-diff.md:25752:A /opt/venv/lib/python3.12/site-packages/timm/data/naflex_random_erasing.py
AppData\Local\Temp\comfyui-diff.md:25753:A /opt/venv/lib/python3.12/site-packages/timm/data/auto_augment.py
AppData\Local\Temp\comfyui-diff.md:25754:A /opt/venv/lib/python3.12/site-packages/timm/data/naflex_dataset.py
AppData\Local\Temp\comfyui-diff.md:25755:A /opt/venv/lib/python3.12/site-packages/timm/data/naflex_transforms.py
AppData\Local\Temp\comfyui-diff.md:25756:A /opt/venv/lib/python3.12/site-packages/timm/data/random_erasing.py
AppData\Local\Temp\comfyui-diff.md:25757:A /opt/venv/lib/python3.12/site-packages/timm/data/tf_preprocessing.py
AppData\Local\Temp\comfyui-diff.md:25758:A /opt/venv/lib/python3.12/site-packages/timm/data/transforms.py
AppData\Local\Temp\comfyui-diff.md:25759:A /opt/venv/lib/python3.12/site-packages/timm/data/transforms_factory.py
AppData\Local\Temp\comfyui-diff.md:25760:A /opt/venv/lib/python3.12/site-packages/timm/data/config.py
AppData\Local\Temp\comfyui-diff.md:25761:A /opt/venv/lib/python3.12/site-packages/timm/data/distributed_sampler.py
AppData\Local\Temp\comfyui-diff.md:25762:A /opt/venv/lib/python3.12/site-packages/timm/data/real_labels.py
AppData\Local\Temp\comfyui-diff.md:25763:A /opt/venv/lib/python3.12/site-packages/timm/layers
AppData\Local\Temp\comfyui-diff.md:25764:A /opt/venv/lib/python3.12/site-packages/timm/layers/eca.py
AppData\Local\Temp\comfyui-diff.md:25765:A /opt/venv/lib/python3.12/site-packages/timm/layers/halo_attn.py
AppData\Local\Temp\comfyui-diff.md:25766:A /opt/venv/lib/python3.12/site-packages/timm/layers/interpolate.py
AppData\Local\Temp\comfyui-diff.md:25767:A /opt/venv/lib/python3.12/site-packages/timm/layers/mlp.py
AppData\Local\Temp\comfyui-diff.md:25768:A /opt/venv/lib/python3.12/site-packages/timm/layers/weight_init.py
AppData\Local\Temp\comfyui-diff.md:25769:A /opt/venv/lib/python3.12/site-packages/timm/layers/adaptive_avgmax_pool.py
AppData\Local\Temp\comfyui-diff.md:25770:A /opt/venv/lib/python3.12/site-packages/timm/layers/coord_attn.py
AppData\Local\Temp\comfyui-diff.md:25771:A /opt/venv/lib/python3.12/site-packages/timm/layers/grid.py
AppData\Local\Temp\comfyui-diff.md:25772:A /opt/venv/lib/python3.12/site-packages/timm/layers/median_pool.py
AppData\Local\Temp\comfyui-diff.md:25773:A /opt/venv/lib/python3.12/site-packages/timm/layers/_fx.py
AppData\Local\Temp\comfyui-diff.md:25774:A /opt/venv/lib/python3.12/site-packages/timm/layers/conv2d_same.py
AppData\Local\Temp\comfyui-diff.md:25775:A /opt/venv/lib/python3.12/site-packages/timm/layers/config.py
AppData\Local\Temp\comfyui-diff.md:25776:A /opt/venv/lib/python3.12/site-packages/timm/layers/create_norm.py
AppData\Local\Temp\comfyui-diff.md:25777:A /opt/venv/lib/python3.12/site-packages/timm/layers/layer_scale.py
AppData\Local\Temp\comfyui-diff.md:25778:A /opt/venv/lib/python3.12/site-packages/timm/layers/norm_act.py
AppData\Local\Temp\comfyui-diff.md:25779:A /opt/venv/lib/python3.12/site-packages/timm/layers/test_time_pool.py
AppData\Local\Temp\comfyui-diff.md:25780:A /opt/venv/lib/python3.12/site-packages/timm/layers/create_act.py
AppData\Local\Temp\comfyui-diff.md:25781:A /opt/venv/lib/python3.12/site-packages/timm/layers/global_context.py
AppData\Local\Temp\comfyui-diff.md:25782:A /opt/venv/lib/python3.12/site-packages/timm/layers/lambda_layer.py
AppData\Local\Temp\comfyui-diff.md:25783:A /opt/venv/lib/python3.12/site-packages/timm/layers/space_to_depth.py
AppData\Local\Temp\comfyui-diff.md:25784:A /opt/venv/lib/python3.12/site-packages/timm/layers/pool1d.py
AppData\Local\Temp\comfyui-diff.md:25785:A /opt/venv/lib/python3.12/site-packages/timm/layers/evo_norm.py
AppData\Local\Temp\comfyui-diff.md:25786:A /opt/venv/lib/python3.12/site-packages/timm/layers/activations_me.py
AppData\Local\Temp\comfyui-diff.md:25787:A /opt/venv/lib/python3.12/site-packages/timm/layers/cond_conv2d.py
AppData\Local\Temp\comfyui-diff.md:25788:A /opt/venv/lib/python3.12/site-packages/timm/layers/format.py
AppData\Local\Temp\comfyui-diff.md:25789:A /opt/venv/lib/python3.12/site-packages/timm/layers/grn.py
AppData\Local\Temp\comfyui-diff.md:25790:A /opt/venv/lib/python3.12/site-packages/timm/layers/helpers.py
AppData\Local\Temp\comfyui-diff.md:25791:A /opt/venv/lib/python3.12/site-packages/timm/layers/non_local_attn.py
AppData\Local\Temp\comfyui-diff.md:25792:A /opt/venv/lib/python3.12/site-packages/timm/layers/__init__.py
AppData\Local\Temp\comfyui-diff.md:25793:A /opt/venv/lib/python3.12/site-packages/timm/layers/activations.py
AppData\Local\Temp\comfyui-diff.md:25794:A /opt/venv/lib/python3.12/site-packages/timm/layers/bottleneck_attn.py
AppData\Local\Temp\comfyui-diff.md:25795:A /opt/venv/lib/python3.12/site-packages/timm/layers/create_norm_act.py
AppData\Local\Temp\comfyui-diff.md:25796:A /opt/venv/lib/python3.12/site-packages/timm/layers/norm.py
AppData\Local\Temp\comfyui-diff.md:25797:A /opt/venv/lib/python3.12/site-packages/timm/layers/pos_embed_rel.py
AppData\Local\Temp\comfyui-diff.md:25798:A /opt/venv/lib/python3.12/site-packages/timm/layers/split_batchnorm.py
AppData\Local\Temp\comfyui-diff.md:25799:A /opt/venv/lib/python3.12/site-packages/timm/layers/trace_utils.py
AppData\Local\Temp\comfyui-diff.md:25800:A /opt/venv/lib/python3.12/site-packages/timm/layers/attention2d.py
AppData\Local\Temp\comfyui-diff.md:25801:A /opt/venv/lib/python3.12/site-packages/timm/layers/fast_norm.py
AppData\Local\Temp\comfyui-diff.md:25802:A /opt/venv/lib/python3.12/site-packages/timm/layers/inplace_abn.py
AppData\Local\Temp\comfyui-diff.md:25803:A /opt/venv/lib/python3.12/site-packages/timm/layers/padding.py
AppData\Local\Temp\comfyui-diff.md:25804:A /opt/venv/lib/python3.12/site-packages/timm/layers/typing.py
AppData\Local\Temp\comfyui-diff.md:25805:A /opt/venv/lib/python3.12/site-packages/timm/layers/classifier.py
AppData\Local\Temp\comfyui-diff.md:25806:A /opt/venv/lib/python3.12/site-packages/timm/layers/other_pool.py
AppData\Local\Temp\comfyui-diff.md:25807:A /opt/venv/lib/python3.12/site-packages/timm/layers/separable_conv.py
AppData\Local\Temp\comfyui-diff.md:25808:A /opt/venv/lib/python3.12/site-packages/timm/layers/cbam.py
AppData\Local\Temp\comfyui-diff.md:25809:A /opt/venv/lib/python3.12/site-packages/timm/layers/conv_bn_act.py
AppData\Local\Temp\comfyui-diff.md:25810:A /opt/venv/lib/python3.12/site-packages/timm/layers/filter_response_norm.py
AppData\Local\Temp\comfyui-diff.md:25811:A /opt/venv/lib/python3.12/site-packages/timm/layers/squeeze_excite.py
AppData\Local\Temp\comfyui-diff.md:25812:A /opt/venv/lib/python3.12/site-packages/timm/layers/attention.py
AppData\Local\Temp\comfyui-diff.md:25813:A /opt/venv/lib/python3.12/site-packages/timm/layers/mixed_conv2d.py
AppData\Local\Temp\comfyui-diff.md:25814:A /opt/venv/lib/python3.12/site-packages/timm/layers/pos_embed_sincos.py
AppData\Local\Temp\comfyui-diff.md:25815:A /opt/venv/lib/python3.12/site-packages/timm/layers/attention_pool.py
AppData\Local\Temp\comfyui-diff.md:25816:A /opt/venv/lib/python3.12/site-packages/timm/layers/attention_pool2d.py
AppData\Local\Temp\comfyui-diff.md:25817:A /opt/venv/lib/python3.12/site-packages/timm/layers/create_conv2d.py
AppData\Local\Temp\comfyui-diff.md:25818:A /opt/venv/lib/python3.12/site-packages/timm/layers/gather_excite.py
AppData\Local\Temp\comfyui-diff.md:25819:A /opt/venv/lib/python3.12/site-packages/timm/layers/hybrid_embed.py
AppData\Local\Temp\comfyui-diff.md:25820:A /opt/venv/lib/python3.12/site-packages/timm/layers/ml_decoder.py
AppData\Local\Temp\comfyui-diff.md:25821:A /opt/venv/lib/python3.12/site-packages/timm/layers/pos_embed.py
AppData\Local\Temp\comfyui-diff.md:25822:A /opt/venv/lib/python3.12/site-packages/timm/layers/std_conv.py
AppData\Local\Temp\comfyui-diff.md:25823:A /opt/venv/lib/python3.12/site-packages/timm/layers/pool2d_same.py
AppData\Local\Temp\comfyui-diff.md:25824:A /opt/venv/lib/python3.12/site-packages/timm/layers/create_attn.py
AppData\Local\Temp\comfyui-diff.md:25825:A /opt/venv/lib/python3.12/site-packages/timm/layers/diff_attention.py
AppData\Local\Temp\comfyui-diff.md:25826:A /opt/venv/lib/python3.12/site-packages/timm/layers/drop.py
AppData\Local\Temp\comfyui-diff.md:25827:A /opt/venv/lib/python3.12/site-packages/timm/layers/patch_dropout.py
AppData\Local\Temp\comfyui-diff.md:25828:A /opt/venv/lib/python3.12/site-packages/timm/layers/split_attn.py
AppData\Local\Temp\comfyui-diff.md:25829:A /opt/venv/lib/python3.12/site-packages/timm/layers/blur_pool.py
AppData\Local\Temp\comfyui-diff.md:25830:A /opt/venv/lib/python3.12/site-packages/timm/layers/linear.py
AppData\Local\Temp\comfyui-diff.md:25831:A /opt/venv/lib/python3.12/site-packages/timm/layers/patch_embed.py
AppData\Local\Temp\comfyui-diff.md:25832:A /opt/venv/lib/python3.12/site-packages/timm/layers/selective_kernel.py
AppData\Local\Temp\comfyui-diff.md:25833:A /opt/venv/lib/python3.12/site-packages/timm/models
AppData\Local\Temp\comfyui-diff.md:25834:A /opt/venv/lib/python3.12/site-packages/timm/models/ghostnet.py
AppData\Local\Temp\comfyui-diff.md:25835:A /opt/venv/lib/python3.12/site-packages/timm/models/repghost.py
AppData\Local\Temp\comfyui-diff.md:25836:A /opt/venv/lib/python3.12/site-packages/timm/models/sequencer.py
AppData\Local\Temp\comfyui-diff.md:25837:A
/opt/venv/lib/python3.12/site-packages/timm/models/vision_transformer_hybrid.py
AppData\Local\Temp\comfyui-diff.md:25838:A /opt/venv/lib/python3.12/site-packages/timm/models/convit.py
AppData\Local\Temp\comfyui-diff.md:25839:A /opt/venv/lib/python3.12/site-packages/timm/models/eva.py
AppData\Local\Temp\comfyui-diff.md:25840:A /opt/venv/lib/python3.12/site-packages/timm/models/hgnet.py
AppData\Local\Temp\comfyui-diff.md:25841:A /opt/venv/lib/python3.12/site-packages/timm/models/iformer.py
AppData\Local\Temp\comfyui-diff.md:25842:A /opt/venv/lib/python3.12/site-packages/timm/models/efficientvim.py
AppData\Local\Temp\comfyui-diff.md:25843:A /opt/venv/lib/python3.12/site-packages/timm/models/senet.py
AppData\Local\Temp\comfyui-diff.md:25844:A /opt/venv/lib/python3.12/site-packages/timm/models/vgg.py
AppData\Local\Temp\comfyui-diff.md:25845:A /opt/venv/lib/python3.12/site-packages/timm/models/volo.py
AppData\Local\Temp\comfyui-diff.md:25846:A /opt/venv/lib/python3.12/site-packages/timm/models/csatv2.py
AppData\Local\Temp\comfyui-diff.md:25847:A /opt/venv/lib/python3.12/site-packages/timm/models/sknet.py
AppData\Local\Temp\comfyui-diff.md:25848:A /opt/venv/lib/python3.12/site-packages/timm/models/swin_transformer_v2_cr.py
AppData\Local\Temp\comfyui-diff.md:25849:A /opt/venv/lib/python3.12/site-packages/timm/models/twins.py
AppData\Local\Temp\comfyui-diff.md:25850:A /opt/venv/lib/python3.12/site-packages/timm/models/cpubone.py
AppData\Local\Temp\comfyui-diff.md:25851:A /opt/venv/lib/python3.12/site-packages/timm/models/mambaout.py
AppData\Local\Temp\comfyui-diff.md:25852:A /opt/venv/lib/python3.12/site-packages/timm/models/shvit.py
AppData\Local\Temp\comfyui-diff.md:25853:A /opt/venv/lib/python3.12/site-packages/timm/models/_efficientnet_blocks.py
AppData\Local\Temp\comfyui-diff.md:25854:A /opt/venv/lib/python3.12/site-packages/timm/models/inception_v3.py
AppData\Local\Temp\comfyui-diff.md:25855:A /opt/venv/lib/python3.12/site-packages/timm/models/swin_transformer.py
AppData\Local\Temp\comfyui-diff.md:25856:A /opt/venv/lib/python3.12/site-packages/timm/models/dla.py
AppData\Local\Temp\comfyui-diff.md:25857:A /opt/venv/lib/python3.12/site-packages/timm/models/nextvit.py
AppData\Local\Temp\comfyui-diff.md:25858:A /opt/venv/lib/python3.12/site-packages/timm/models/resnest.py
AppData\Local\Temp\comfyui-diff.md:25859:A /opt/venv/lib/python3.12/site-packages/timm/models/__init__.py
AppData\Local\Temp\comfyui-diff.md:25860:A /opt/venv/lib/python3.12/site-packages/timm/models/hub.py
AppData\Local\Temp\comfyui-diff.md:25861:A /opt/venv/lib/python3.12/site-packages/timm/models/levit.py
AppData\Local\Temp\comfyui-diff.md:25862:A /opt/venv/lib/python3.12/site-packages/timm/models/mvitv2.py
AppData\Local\Temp\comfyui-diff.md:25863:A /opt/venv/lib/python3.12/site-packages/timm/models/_factory.py
AppData\Local\Temp\comfyui-diff.md:25864:A /opt/venv/lib/python3.12/site-packages/timm/models/_pruned
AppData\Local\Temp\comfyui-diff.md:25865:A
/opt/venv/lib/python3.12/site-packages/timm/models/_pruned/ecaresnet101d_pruned.txt
AppData\Local\Temp\comfyui-diff.md:25866:A
/opt/venv/lib/python3.12/site-packages/timm/models/_pruned/ecaresnet50d_pruned.txt
AppData\Local\Temp\comfyui-diff.md:25867:A
/opt/venv/lib/python3.12/site-packages/timm/models/_pruned/efficientnet_b1_pruned.txt
AppData\Local\Temp\comfyui-diff.md:25868:A
/opt/venv/lib/python3.12/site-packages/timm/models/_pruned/efficientnet_b2_pruned.txt
AppData\Local\Temp\comfyui-diff.md:25869:A
/opt/venv/lib/python3.12/site-packages/timm/models/_pruned/efficientnet_b3_pruned.txt
AppData\Local\Temp\comfyui-diff.md:25870:A /opt/venv/lib/python3.12/site-packages/timm/models/convnext.py
AppData\Local\Temp\comfyui-diff.md:25871:A /opt/venv/lib/python3.12/site-packages/timm/models/focalnet.py
AppData\Local\Temp\comfyui-diff.md:25872:A /opt/venv/lib/python3.12/site-packages/timm/models/nest.py
AppData\Local\Temp\comfyui-diff.md:25873:A /opt/venv/lib/python3.12/site-packages/timm/models/convmixer.py
AppData\Local\Temp\comfyui-diff.md:25874:A /opt/venv/lib/python3.12/site-packages/timm/models/selecsls.py
AppData\Local\Temp\comfyui-diff.md:25875:A /opt/venv/lib/python3.12/site-packages/timm/models/tiny_vit.py
AppData\Local\Temp\comfyui-diff.md:25876:A /opt/venv/lib/python3.12/site-packages/timm/models/vovnet.py
AppData\Local\Temp\comfyui-diff.md:25877:A /opt/venv/lib/python3.12/site-packages/timm/models/efficientvit_mit.py
AppData\Local\Temp\comfyui-diff.md:25878:A /opt/venv/lib/python3.12/site-packages/timm/models/fastvit.py
AppData\Local\Temp\comfyui-diff.md:25879:A /opt/venv/lib/python3.12/site-packages/timm/models/regnet.py
AppData\Local\Temp\comfyui-diff.md:25880:A /opt/venv/lib/python3.12/site-packages/timm/models/res2net.py
AppData\Local\Temp\comfyui-diff.md:25881:A
/opt/venv/lib/python3.12/site-packages/timm/models/vision_transformer_relpos.py
AppData\Local\Temp\comfyui-diff.md:25882:A /opt/venv/lib/python3.12/site-packages/timm/models/xception.py
AppData\Local\Temp\comfyui-diff.md:25883:A /opt/venv/lib/python3.12/site-packages/timm/models/crossvit.py
AppData\Local\Temp\comfyui-diff.md:25884:A /opt/venv/lib/python3.12/site-packages/timm/models/efficientformer.py
AppData\Local\Temp\comfyui-diff.md:25885:A /opt/venv/lib/python3.12/site-packages/timm/models/efficientnet.py
AppData\Local\Temp\comfyui-diff.md:25886:A /opt/venv/lib/python3.12/site-packages/timm/models/naflexvit.py
AppData\Local\Temp\comfyui-diff.md:25887:A /opt/venv/lib/python3.12/site-packages/timm/models/pvt_v2.py
AppData\Local\Temp\comfyui-diff.md:25888:A /opt/venv/lib/python3.12/site-packages/timm/models/vision_transformer_sam.py
AppData\Local\Temp\comfyui-diff.md:25889:A /opt/venv/lib/python3.12/site-packages/timm/models/features.py
AppData\Local\Temp\comfyui-diff.md:25890:A /opt/venv/lib/python3.12/site-packages/timm/models/pnasnet.py
AppData\Local\Temp\comfyui-diff.md:25891:A /opt/venv/lib/python3.12/site-packages/timm/models/qwen3_vit.py
AppData\Local\Temp\comfyui-diff.md:25892:A /opt/venv/lib/python3.12/site-packages/timm/models/deit.py
AppData\Local\Temp\comfyui-diff.md:25893:A /opt/venv/lib/python3.12/site-packages/timm/models/hiera.py
AppData\Local\Temp\comfyui-diff.md:25894:A /opt/venv/lib/python3.12/site-packages/timm/models/inception_v4.py
AppData\Local\Temp\comfyui-diff.md:25895:A /opt/venv/lib/python3.12/site-packages/timm/models/beit.py
AppData\Local\Temp\comfyui-diff.md:25896:A /opt/venv/lib/python3.12/site-packages/timm/models/deepseek_vit.py
AppData\Local\Temp\comfyui-diff.md:25897:A /opt/venv/lib/python3.12/site-packages/timm/models/factory.py
AppData\Local\Temp\comfyui-diff.md:25898:A /opt/venv/lib/python3.12/site-packages/timm/models/mobilenetv3.py
AppData\Local\Temp\comfyui-diff.md:25899:A /opt/venv/lib/python3.12/site-packages/timm/models/nfnet.py
AppData\Local\Temp\comfyui-diff.md:25900:A /opt/venv/lib/python3.12/site-packages/timm/models/rdnet.py
AppData\Local\Temp\comfyui-diff.md:25901:A /opt/venv/lib/python3.12/site-packages/timm/models/xcit.py
AppData\Local\Temp\comfyui-diff.md:25902:A /opt/venv/lib/python3.12/site-packages/timm/models/_hub.py
AppData\Local\Temp\comfyui-diff.md:25903:A /opt/venv/lib/python3.12/site-packages/timm/models/_manipulate.py
AppData\Local\Temp\comfyui-diff.md:25904:A /opt/venv/lib/python3.12/site-packages/timm/models/swin_transformer_v2.py
AppData\Local\Temp\comfyui-diff.md:25905:A /opt/venv/lib/python3.12/site-packages/timm/models/cait.py
AppData\Local\Temp\comfyui-diff.md:25906:A /opt/venv/lib/python3.12/site-packages/timm/models/gcvit.py
AppData\Local\Temp\comfyui-diff.md:25907:A /opt/venv/lib/python3.12/site-packages/timm/models/resnetv2.py
AppData\Local\Temp\comfyui-diff.md:25908:A /opt/venv/lib/python3.12/site-packages/timm/models/_helpers.py
AppData\Local\Temp\comfyui-diff.md:25909:A /opt/venv/lib/python3.12/site-packages/timm/models/inception_resnet_v2.py
AppData\Local\Temp\comfyui-diff.md:25910:A /opt/venv/lib/python3.12/site-packages/timm/models/densenet.py
AppData\Local\Temp\comfyui-diff.md:25911:A /opt/venv/lib/python3.12/site-packages/timm/models/lcnetv2.py
AppData\Local\Temp\comfyui-diff.md:25912:A /opt/venv/lib/python3.12/site-packages/timm/models/mlp_mixer.py
AppData\Local\Temp\comfyui-diff.md:25913:A /opt/venv/lib/python3.12/site-packages/timm/models/vitamin.py
AppData\Local\Temp\comfyui-diff.md:25914:A /opt/venv/lib/python3.12/site-packages/timm/models/_builder.py
AppData\Local\Temp\comfyui-diff.md:25915:A /opt/venv/lib/python3.12/site-packages/timm/models/hieradet_sam2.py
AppData\Local\Temp\comfyui-diff.md:25916:A /opt/venv/lib/python3.12/site-packages/timm/models/maxxvit.py
AppData\Local\Temp\comfyui-diff.md:25917:A /opt/venv/lib/python3.12/site-packages/timm/models/metaformer.py
AppData\Local\Temp\comfyui-diff.md:25918:A /opt/venv/lib/python3.12/site-packages/timm/models/pit.py
AppData\Local\Temp\comfyui-diff.md:25919:A /opt/venv/lib/python3.12/site-packages/timm/models/_efficientnet_builder.py
AppData\Local\Temp\comfyui-diff.md:25920:A /opt/venv/lib/python3.12/site-packages/timm/models/_prune.py
AppData\Local\Temp\comfyui-diff.md:25921:A /opt/venv/lib/python3.12/site-packages/timm/models/coat.py
AppData\Local\Temp\comfyui-diff.md:25922:A /opt/venv/lib/python3.12/site-packages/timm/models/vision_transformer.py
AppData\Local\Temp\comfyui-diff.md:25923:A /opt/venv/lib/python3.12/site-packages/timm/models/starnet.py
AppData\Local\Temp\comfyui-diff.md:25924:A /opt/venv/lib/python3.12/site-packages/timm/models/helpers.py
AppData\Local\Temp\comfyui-diff.md:25925:A /opt/venv/lib/python3.12/site-packages/timm/models/hrnet.py
AppData\Local\Temp\comfyui-diff.md:25926:A /opt/venv/lib/python3.12/site-packages/timm/models/_pretrained.py
AppData\Local\Temp\comfyui-diff.md:25927:A /opt/venv/lib/python3.12/site-packages/timm/models/fasternet.py
AppData\Local\Temp\comfyui-diff.md:25928:A /opt/venv/lib/python3.12/site-packages/timm/models/repvit.py
AppData\Local\Temp\comfyui-diff.md:25929:A /opt/venv/lib/python3.12/site-packages/timm/models/resnet.py
AppData\Local\Temp\comfyui-diff.md:25930:A /opt/venv/lib/python3.12/site-packages/timm/models/xception_aligned.py
AppData\Local\Temp\comfyui-diff.md:25931:A /opt/venv/lib/python3.12/site-packages/timm/models/_features.py
AppData\Local\Temp\comfyui-diff.md:25932:A /opt/venv/lib/python3.12/site-packages/timm/models/byobnet.py
AppData\Local\Temp\comfyui-diff.md:25933:A /opt/venv/lib/python3.12/site-packages/timm/models/gemma4_vit.py
AppData\Local\Temp\comfyui-diff.md:25934:A /opt/venv/lib/python3.12/site-packages/timm/models/hardcorenas.py


PS C:\Users\hoang> Select-String `
>>   -Path "$env:TEMP\comfyui-diff.md" `
>>   -Pattern 'torch|nvidia|cuda|rocm|hip' `
>>   -CaseSensitive:$false |
>>   Select-Object -First 300

AppData\Local\Temp\comfyui-diff.md:45:rocm/comfyui:comfyui-0.18.2.amd0_rocm7.2.0_ubuntu24.04   c88f20157d88
67.6GB             0B
AppData\Local\Temp\comfyui-diff.md:46:PS C:\Users\hoang> wsl docker run --rm --device=/dev/dxg --ipc=host
--shm-size=8G --cap-add=SYS_PTRACE --security-opt seccomp=unconfined -v
/usr/lib/wsl/lib/libdxcore.so:/usr/lib/libdxcore.so -v /opt/rocm/lib/librocdxg.so:/usr/lib/librocdxg.so -v
/opt/rocm/share/rocdxg/dids.conf:/usr/share/rocdxg/dids.conf -e HSA_ENABLE_DXG_DETECTION=1
rocm/comfyui:comfyui-0.18.2.amd0_rocm7.2.0_ubuntu24.04 python -c "import torch; print('Torch:', torch.__version__);
print('HIP:', torch.version.hip); print('Available:', torch.cuda.is_available()); print('Count:',
torch.cuda.device_count()); print('GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NONE')"
AppData\Local\Temp\comfyui-diff.md:47:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:971: UserWarning:
Can't initialize amdsmi - Error code: 34
AppData\Local\Temp\comfyui-diff.md:49:Torch: 2.10.0a0+git449b176
AppData\Local\Temp\comfyui-diff.md:50:HIP: 7.2.26015
AppData\Local\Temp\comfyui-diff.md:114:PS C:\Users\hoang> wsl ls -l /opt/rocm/lib/librocdxg.so
AppData\Local\Temp\comfyui-diff.md:115:lrwxrwxrwx 1 root root 14 Aug  4 22:55 /opt/rocm/lib/librocdxg.so ->
librocdxg.so.1
AppData\Local\Temp\comfyui-diff.md:119:b6cf5b7d8494   rocm/comfyui:comfyui-0.18.2.amd0_rocm7.2.0_ubuntu24.04   "python
/workload/Co…"   8 minutes ago   Exited (1) 24 seconds ago             comfyui
AppData\Local\Temp\comfyui-diff.md:123:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:125:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:126:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:127:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:128:comfy-aimdo failed to load: libcuda.so.1: cannot open shared object file: No
such file or directory
AppData\Local\Temp\comfyui-diff.md:129:NOTE: comfy-aimdo is currently only support for Nvidia GPUs
AppData\Local\Temp\comfyui-diff.md:153:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:68:
FutureWarning: The pynvml package is deprecated. Please install nvidia-ml-py instead. If you did not install pynvml
directly, please report this to the maintainers of the package that installed pynvml for you.
AppData\Local\Temp\comfyui-diff.md:155:Found comfy_kitchen backend cuda: {'available': False, 'disabled': False,
'unavailable_reason': 'CUDA not available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:156:Found comfy_kitchen backend triton: {'available': False, 'disabled': True,
'unavailable_reason': 'Neither CUDA nor XPU available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:165:    total_vram = get_total_memory(get_torch_device()) / (1024 * 1024)
AppData\Local\Temp\comfyui-diff.md:167:  File "/workload/ComfyUI/comfy/model_management.py", line 207, in
get_torch_device
AppData\Local\Temp\comfyui-diff.md:168:    return torch.device(torch.cuda.current_device())
AppData\Local\Temp\comfyui-diff.md:170:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:172:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:173:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:174:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:175:comfy-aimdo failed to load: libcuda.so.1: cannot open shared object file: No
such file or directory
AppData\Local\Temp\comfyui-diff.md:176:NOTE: comfy-aimdo is currently only support for Nvidia GPUs
AppData\Local\Temp\comfyui-diff.md:200:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:68:
FutureWarning: The pynvml package is deprecated. Please install nvidia-ml-py instead. If you did not install pynvml
directly, please report this to the maintainers of the package that installed pynvml for you.
AppData\Local\Temp\comfyui-diff.md:203:Found comfy_kitchen backend cuda: {'available': False, 'disabled': False,
'unavailable_reason': 'CUDA not available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:204:Found comfy_kitchen backend triton: {'available': False, 'disabled': True,
'unavailable_reason': 'Neither CUDA nor XPU available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:212:    total_vram = get_total_memory(get_torch_device()) / (1024 * 1024)
AppData\Local\Temp\comfyui-diff.md:214:  File "/workload/ComfyUI/comfy/model_management.py", line 207, in
get_torch_device
AppData\Local\Temp\comfyui-diff.md:215:    return torch.device(torch.cuda.current_device())
AppData\Local\Temp\comfyui-diff.md:217:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:219:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:220:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:221:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:229:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:231:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:232:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:233:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:234:comfy-aimdo failed to load: libcuda.so.1: cannot open shared object file: No
such file or directory
AppData\Local\Temp\comfyui-diff.md:235:NOTE: comfy-aimdo is currently only support for Nvidia GPUs
AppData\Local\Temp\comfyui-diff.md:259:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:68:
FutureWarning: The pynvml package is deprecated. Please install nvidia-ml-py instead. If you did not install pynvml
directly, please report this to the maintainers of the package that installed pynvml for you.
AppData\Local\Temp\comfyui-diff.md:262:Found comfy_kitchen backend cuda: {'available': False, 'disabled': False,
'unavailable_reason': 'CUDA not available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:263:Found comfy_kitchen backend triton: {'available': False, 'disabled': True,
'unavailable_reason': 'Neither CUDA nor XPU available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:271:    total_vram = get_total_memory(get_torch_device()) / (1024 * 1024)
AppData\Local\Temp\comfyui-diff.md:273:  File "/workload/ComfyUI/comfy/model_management.py", line 207, in
get_torch_device
AppData\Local\Temp\comfyui-diff.md:274:    return torch.device(torch.cuda.current_device())
AppData\Local\Temp\comfyui-diff.md:276:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:278:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:279:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:280:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:281:comfy-aimdo failed to load: libcuda.so.1: cannot open shared object file: No
such file or directory
AppData\Local\Temp\comfyui-diff.md:282:NOTE: comfy-aimdo is currently only support for Nvidia GPUs
AppData\Local\Temp\comfyui-diff.md:306:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:68:
FutureWarning: The pynvml package is deprecated. Please install nvidia-ml-py instead. If you did not install pynvml
directly, please report this to the maintainers of the package that installed pynvml for you.
AppData\Local\Temp\comfyui-diff.md:308:Found comfy_kitchen backend triton: {'available': False, 'disabled': True,
'unavailable_reason': 'Neither CUDA nor XPU available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:309:Found comfy_kitchen backend cuda: {'available': False, 'disabled': False,
'unavailable_reason': 'CUDA not available on this system', 'capabilities': []}
AppData\Local\Temp\comfyui-diff.md:318:    total_vram = get_total_memory(get_torch_device()) / (1024 * 1024)
AppData\Local\Temp\comfyui-diff.md:320:  File "/workload/ComfyUI/comfy/model_management.py", line 207, in
get_torch_device
AppData\Local\Temp\comfyui-diff.md:321:    return torch.device(torch.cuda.current_device())
AppData\Local\Temp\comfyui-diff.md:323:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
1267, in current_device
AppData\Local\Temp\comfyui-diff.md:325:  File "/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py", line
591, in _lazy_init
AppData\Local\Temp\comfyui-diff.md:326:    torch._C._cuda_init()
AppData\Local\Temp\comfyui-diff.md:327:RuntimeError: Found no NVIDIA driver on your system. Please check that you have
an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx
AppData\Local\Temp\comfyui-diff.md:333:PATH=/opt/venv/bin:/opt/rocm-7.2.0/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:
/usr/bin:/sbin:/bin
AppData\Local\Temp\comfyui-diff.md:335:ROCM_PATH=/opt/rocm-7.2.0
AppData\Local\Temp\comfyui-diff.md:337:PYTORCH_ROCM_ARCH=gfx942;gfx950
AppData\Local\Temp\comfyui-diff.md:341:NVTE_USE_HIPBLASLT=1
AppData\Local\Temp\comfyui-diff.md:342:NVTE_FRAMEWORK=pytorch
AppData\Local\Temp\comfyui-diff.md:343:NVTE_ROCM_ARCH=gfx942;gfx950
AppData\Local\Temp\comfyui-diff.md:350:NVTE_USE_ROCM=1
AppData\Local\Temp\comfyui-diff.md:354:HIP_ARCHITECTURES=gfx942,gfx950
AppData\Local\Temp\comfyui-diff.md:356:BUILD_ROCM_VERSION=7.2
AppData\Local\Temp\comfyui-diff.md:357:FBGEMM_TBE_ROCM_HIP_BACKWARD_KERNEL=1
AppData\Local\Temp\comfyui-diff.md:358:ROCM_VERSION=72000
AppData\Local\Temp\comfyui-diff.md:359:HIPBLAS_V2=1
AppData\Local\Temp\comfyui-diff.md:368:PS C:\Users\hoang> wsl docker run --rm -it --device=/dev/dxg --ipc=host
--shm-size=8G --cap-add=SYS_PTRACE --security-opt seccomp=unconfined -v
/usr/lib/wsl/lib/libdxcore.so:/usr/lib/libdxcore.so -v /opt/rocm/lib/librocdxg.so:/usr/lib/librocdxg.so -v
/opt/rocm/share/rocdxg/dids.conf:/usr/share/rocdxg/dids.conf -e HSA_ENABLE_DXG_DETECTION=1
rocm/comfyui:comfyui-0.18.2.amd0_rocm7.2.0_ubuntu24.04 bash
AppData\Local\Temp\comfyui-diff.md:369:root@590872cfe5d4:/workspace# /opt/venv/bin/python -c "import torch;
print(torch.__version__); print('HIP:', torch.version.hip); print('CUDA:', torch.version.cuda); print('available:',
torch.cuda.is_available()); print('count:', torch.cuda.device_count())"
AppData\Local\Temp\comfyui-diff.md:371:HIP: 7.2.26015
AppData\Local\Temp\comfyui-diff.md:372:CUDA: None
AppData\Local\Temp\comfyui-diff.md:374:/opt/venv/lib/python3.12/site-packages/torch/cuda/__init__.py:971: UserWarning:
Can't initialize amdsmi - Error code: 34
AppData\Local\Temp\comfyui-diff.md:381:PATH=/opt/venv/bin:/opt/rocm-7.2.0/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:
/usr/bin:/sbin:/bin
AppData\Local\Temp\comfyui-diff.md:383:ROCM_PATH=/opt/rocm-7.2.0
AppData\Local\Temp\comfyui-diff.md:385:PYTORCH_ROCM_ARCH=gfx942;gfx950
AppData\Local\Temp\comfyui-diff.md:389:NVTE_USE_HIPBLASLT=1
AppData\Local\Temp\comfyui-diff.md:390:NVTE_FRAMEWORK=pytorch
AppData\Local\Temp\comfyui-diff.md:391:NVTE_ROCM_ARCH=gfx942;gfx950
AppData\Local\Temp\comfyui-diff.md:398:NVTE_USE_ROCM=1
AppData\Local\Temp\comfyui-diff.md:402:HIP_ARCHITECTURES=gfx942,gfx950
AppData\Local\Temp\comfyui-diff.md:404:BUILD_ROCM_VERSION=7.2
AppData\Local\Temp\comfyui-diff.md:405:FBGEMM_TBE_ROCM_HIP_BACKWARD_KERNEL=1
AppData\Local\Temp\comfyui-diff.md:406:ROCM_VERSION=72000
AppData\Local\Temp\comfyui-diff.md:407:HIPBLAS_V2=1
AppData\Local\Temp\comfyui-diff.md:446:A /root/.cache/uv/simple-v21/pypi/nvidia-nvtx.rkyv
AppData\Local\Temp\comfyui-diff.md:464:A /root/.cache/uv/simple-v21/pypi/nvidia-cuda-cupti.rkyv
AppData\Local\Temp\comfyui-diff.md:468:A /root/.cache/uv/simple-v21/pypi/nvidia-cusparselt-cu13.rkyv
AppData\Local\Temp\comfyui-diff.md:469:A /root/.cache/uv/simple-v21/pypi/nvidia-nccl-cu13.rkyv
AppData\Local\Temp\comfyui-diff.md:473:A /root/.cache/uv/simple-v21/pypi/nvidia-cudnn-cu13.rkyv
AppData\Local\Temp\comfyui-diff.md:474:A /root/.cache/uv/simple-v21/pypi/nvidia-cufft.rkyv
AppData\Local\Temp\comfyui-diff.md:480:A /root/.cache/uv/simple-v21/pypi/nvidia-cublas.rkyv
AppData\Local\Temp\comfyui-diff.md:481:A /root/.cache/uv/simple-v21/pypi/nvidia-cuda-runtime.rkyv
AppData\Local\Temp\comfyui-diff.md:483:A /root/.cache/uv/simple-v21/pypi/torchvision.rkyv
AppData\Local\Temp\comfyui-diff.md:489:A /root/.cache/uv/simple-v21/pypi/nvidia-ml-py.rkyv
AppData\Local\Temp\comfyui-diff.md:494:A /root/.cache/uv/simple-v21/pypi/nvidia-cusparse.rkyv
AppData\Local\Temp\comfyui-diff.md:502:A /root/.cache/uv/simple-v21/pypi/cuda-toolkit.rkyv
AppData\Local\Temp\comfyui-diff.md:509:A /root/.cache/uv/simple-v21/pypi/cuda-bindings.rkyv
AppData\Local\Temp\comfyui-diff.md:513:A /root/.cache/uv/simple-v21/pypi/nvidia-curand.rkyv
AppData\Local\Temp\comfyui-diff.md:517:A /root/.cache/uv/simple-v21/pypi/nvidia-nvjitlink.rkyv
AppData\Local\Temp\comfyui-diff.md:518:A /root/.cache/uv/simple-v21/pypi/nvidia-nvshmem-cu13.rkyv
AppData\Local\Temp\comfyui-diff.md:527:A /root/.cache/uv/simple-v21/pypi/nvidia-cusolver.rkyv
AppData\Local\Temp\comfyui-diff.md:532:A /root/.cache/uv/simple-v21/pypi/nvidia-cuda-nvrtc.rkyv
AppData\Local\Temp\comfyui-diff.md:541:A /root/.cache/uv/simple-v21/pypi/nvidia-cufile.rkyv
AppData\Local\Temp\comfyui-diff.md:544:A /root/.cache/uv/simple-v21/pypi/torch.rkyv
AppData\Local\Temp\comfyui-diff.md:546:A /root/.cache/uv/simple-v21/pypi/cuda-pathfinder.rkyv
AppData\Local\Temp\comfyui-diff.md:562:A /root/.cache/uv/wheels-v6/pypi/nvidia-cublas
AppData\Local\Temp\comfyui-diff.md:563:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cublas/13.1.1.3-py3-none-manylinux_2_27_x86_64
AppData\Local\Temp\comfyui-diff.md:564:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cublas/13.1.1.3-py3-none-manylinux_2_27_x86_64.http
AppData\Local\Temp\comfyui-diff.md:565:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cublas/13.1.1.3-py3-none-manylinux_2_27_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:566:A /root/.cache/uv/wheels-v6/pypi/nvidia-cuda-cupti
AppData\Local\Temp\comfyui-diff.md:567:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-cupti/13.0.85-py3-none-manylinux_2_25_x86_64
AppData\Local\Temp\comfyui-diff.md:568:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-cupti/13.0.85-py3-none-manylinux_2_25_x86_64.http
AppData\Local\Temp\comfyui-diff.md:569:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-cupti/13.0.85-py3-none-manylinux_2_25_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:598:A /root/.cache/uv/wheels-v6/pypi/nvidia-cusparselt-cu13
AppData\Local\Temp\comfyui-diff.md:599:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparselt-cu13/0.8.1-py3-none-manylinux2014_x86_64
AppData\Local\Temp\comfyui-diff.md:600:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparselt-cu13/0.8.1-py3-none-manylinux2014_x86_64.http
AppData\Local\Temp\comfyui-diff.md:601:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparselt-cu13/0.8.1-py3-none-manylinux2014_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:606:A /root/.cache/uv/wheels-v6/pypi/cuda-bindings
AppData\Local\Temp\comfyui-diff.md:607:A
/root/.cache/uv/wheels-v6/pypi/cuda-bindings/13.4.3-cp312-cp312-manylinux_2_24_x86_64.manylinux_2_28_x86_64
AppData\Local\Temp\comfyui-diff.md:608:A
/root/.cache/uv/wheels-v6/pypi/cuda-bindings/13.4.3-cp312-cp312-manylinux_2_24_x86_64.manylinux_2_28_x86_64.http
AppData\Local\Temp\comfyui-diff.md:609:A
/root/.cache/uv/wheels-v6/pypi/cuda-bindings/13.4.3-cp312-cp312-manylinux_2_24_x86_64.manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:618:A /root/.cache/uv/wheels-v6/pypi/nvidia-nccl-cu13
AppData\Local\Temp\comfyui-diff.md:619:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nccl-cu13/2.30.7-py3-none-manylinux_2_18_x86_64
AppData\Local\Temp\comfyui-diff.md:620:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nccl-cu13/2.30.7-py3-none-manylinux_2_18_x86_64.http
AppData\Local\Temp\comfyui-diff.md:621:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nccl-cu13/2.30.7-py3-none-manylinux_2_18_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:638:A /root/.cache/uv/wheels-v6/pypi/nvidia-cusparse
AppData\Local\Temp\comfyui-diff.md:639:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparse/12.6.3.3-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64
AppData\Local\Temp\comfyui-diff.md:640:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparse/12.6.3.3-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.http
AppData\Local\Temp\comfyui-diff.md:641:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusparse/12.6.3.3-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:646:A /root/.cache/uv/wheels-v6/pypi/nvidia-cufft
AppData\Local\Temp\comfyui-diff.md:647:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufft/12.0.0.61-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:648:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufft/12.0.0.61-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64
AppData\Local\Temp\comfyui-diff.md:649:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufft/12.0.0.61-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.http
AppData\Local\Temp\comfyui-diff.md:666:A /root/.cache/uv/wheels-v6/pypi/nvidia-cuda-runtime
AppData\Local\Temp\comfyui-diff.md:667:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-runtime/13.0.96-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64
AppData\Local\Temp\comfyui-diff.md:668:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-runtime/13.0.96-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.http
AppData\Local\Temp\comfyui-diff.md:669:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-runtime/13.0.96-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:678:A /root/.cache/uv/wheels-v6/pypi/cuda-pathfinder
AppData\Local\Temp\comfyui-diff.md:679:A /root/.cache/uv/wheels-v6/pypi/cuda-pathfinder/1.8.3-py3-none-any
AppData\Local\Temp\comfyui-diff.md:680:A /root/.cache/uv/wheels-v6/pypi/cuda-pathfinder/1.8.3-py3-none-any.http
AppData\Local\Temp\comfyui-diff.md:681:A /root/.cache/uv/wheels-v6/pypi/cuda-pathfinder/1.8.3-py3-none-any.msgpack
AppData\Local\Temp\comfyui-diff.md:686:A /root/.cache/uv/wheels-v6/pypi/nvidia-curand
AppData\Local\Temp\comfyui-diff.md:687:A
/root/.cache/uv/wheels-v6/pypi/nvidia-curand/10.4.0.35-py3-none-manylinux_2_27_x86_64
AppData\Local\Temp\comfyui-diff.md:688:A
/root/.cache/uv/wheels-v6/pypi/nvidia-curand/10.4.0.35-py3-none-manylinux_2_27_x86_64.http
AppData\Local\Temp\comfyui-diff.md:689:A
/root/.cache/uv/wheels-v6/pypi/nvidia-curand/10.4.0.35-py3-none-manylinux_2_27_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:694:A /root/.cache/uv/wheels-v6/pypi/torchvision
AppData\Local\Temp\comfyui-diff.md:695:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.26.0-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:696:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.27.0-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:697:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.27.1-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:698:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.28.0-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:699:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.29.0-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:700:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.29.1-cp312-cp312-manylinux_2_28_x86_64
AppData\Local\Temp\comfyui-diff.md:701:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.29.1-cp312-cp312-manylinux_2_28_x86_64.http
AppData\Local\Temp\comfyui-diff.md:702:A
/root/.cache/uv/wheels-v6/pypi/torchvision/0.29.1-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:711:A /root/.cache/uv/wheels-v6/pypi/nvidia-cusolver
AppData\Local\Temp\comfyui-diff.md:712:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusolver/12.0.4.66-py3-none-manylinux_2_27_x86_64
AppData\Local\Temp\comfyui-diff.md:713:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusolver/12.0.4.66-py3-none-manylinux_2_27_x86_64.http
AppData\Local\Temp\comfyui-diff.md:714:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cusolver/12.0.4.66-py3-none-manylinux_2_27_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:727:A /root/.cache/uv/wheels-v6/pypi/cuda-toolkit
AppData\Local\Temp\comfyui-diff.md:728:A /root/.cache/uv/wheels-v6/pypi/cuda-toolkit/13.0.3.0-py2.py3-none-any.msgpack
AppData\Local\Temp\comfyui-diff.md:729:A /root/.cache/uv/wheels-v6/pypi/cuda-toolkit/13.0.3.0-py2.py3-none-any
AppData\Local\Temp\comfyui-diff.md:730:A /root/.cache/uv/wheels-v6/pypi/cuda-toolkit/13.0.3.0-py2.py3-none-any.http
AppData\Local\Temp\comfyui-diff.md:731:A /root/.cache/uv/wheels-v6/pypi/nvidia-cudnn-cu13
AppData\Local\Temp\comfyui-diff.md:732:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cudnn-cu13/9.24.0.43-py3-none-manylinux_2_27_x86_64
AppData\Local\Temp\comfyui-diff.md:733:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cudnn-cu13/9.24.0.43-py3-none-manylinux_2_27_x86_64.http
AppData\Local\Temp\comfyui-diff.md:734:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cudnn-cu13/9.24.0.43-py3-none-manylinux_2_27_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:735:A /root/.cache/uv/wheels-v6/pypi/nvidia-nvtx
AppData\Local\Temp\comfyui-diff.md:736:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvtx/13.0.85-py3-none-manylinux1_x86_64.manylinux_2_5_x86_64
AppData\Local\Temp\comfyui-diff.md:737:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvtx/13.0.85-py3-none-manylinux1_x86_64.manylinux_2_5_x86_64.http
AppData\Local\Temp\comfyui-diff.md:738:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvtx/13.0.85-py3-none-manylinux1_x86_64.manylinux_2_5_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:747:A /root/.cache/uv/wheels-v6/pypi/nvidia-cuda-nvrtc
AppData\Local\Temp\comfyui-diff.md:748:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-nvrtc/13.0.88-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:749:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-nvrtc/13.0.88-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64
AppData\Local\Temp\comfyui-diff.md:750:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cuda-nvrtc/13.0.88-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64.http
AppData\Local\Temp\comfyui-diff.md:763:A /root/.cache/uv/wheels-v6/pypi/torch
AppData\Local\Temp\comfyui-diff.md:764:A
/root/.cache/uv/wheels-v6/pypi/torch/2.14.1-cp312-cp312-manylinux_2_28_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:765:A /root/.cache/uv/wheels-v6/pypi/torch/2.14.1-cp312-cp312-manylinux_2_28_x86_64
AppData\Local\Temp\comfyui-diff.md:766:A
/root/.cache/uv/wheels-v6/pypi/torch/2.14.1-cp312-cp312-manylinux_2_28_x86_64.http
AppData\Local\Temp\comfyui-diff.md:771:A /root/.cache/uv/wheels-v6/pypi/nvidia-nvshmem-cu13
AppData\Local\Temp\comfyui-diff.md:772:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvshmem-cu13/3.4.5-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.http
AppData\Local\Temp\comfyui-diff.md:773:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvshmem-cu13/3.4.5-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:774:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvshmem-cu13/3.4.5-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64
AppData\Local\Temp\comfyui-diff.md:783:A /root/.cache/uv/wheels-v6/pypi/nvidia-cufile
AppData\Local\Temp\comfyui-diff.md:784:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufile/1.15.1.6-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64
AppData\Local\Temp\comfyui-diff.md:785:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufile/1.15.1.6-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.http
AppData\Local\Temp\comfyui-diff.md:786:A
/root/.cache/uv/wheels-v6/pypi/nvidia-cufile/1.15.1.6-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:787:A /root/.cache/uv/wheels-v6/pypi/nvidia-nvjitlink
AppData\Local\Temp\comfyui-diff.md:788:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvjitlink/13.4.92-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64
AppData\Local\Temp\comfyui-diff.md:789:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvjitlink/13.4.92-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64.http
AppData\Local\Temp\comfyui-diff.md:790:A
/root/.cache/uv/wheels-v6/pypi/nvidia-nvjitlink/13.4.92-py3-none-manylinux2010_x86_64.manylinux_2_12_x86_64.msgpack
AppData\Local\Temp\comfyui-diff.md:1138:A /root/.cache/uv/archive-v0/9Y57UbtIjSZWpSdr0Gg6z/timm/utils/cuda.py
AppData\Local\Temp\comfyui-diff.md:1173:A /root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia
AppData\Local\Temp\comfyui-diff.md:1174:A /root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn
AppData\Local\Temp\comfyui-diff.md:1175:A /root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include
AppData\Local\Temp\comfyui-diff.md:1176:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_adv_v9.h
AppData\Local\Temp\comfyui-diff.md:1177:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_ops_v9.h
AppData\Local\Temp\comfyui-diff.md:1178:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_v9.h
AppData\Local\Temp\comfyui-diff.md:1179:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_version.h
AppData\Local\Temp\comfyui-diff.md:1180:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_backend.h
AppData\Local\Temp\comfyui-diff.md:1181:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_backend_v9.h
AppData\Local\Temp\comfyui-diff.md:1182:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_cnn.h
AppData\Local\Temp\comfyui-diff.md:1183:A /root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn.h
AppData\Local\Temp\comfyui-diff.md:1184:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_adv.h
AppData\Local\Temp\comfyui-diff.md:1185:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_cnn_v9.h
AppData\Local\Temp\comfyui-diff.md:1186:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_graph.h
AppData\Local\Temp\comfyui-diff.md:1187:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_graph_v9.h
AppData\Local\Temp\comfyui-diff.md:1188:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_version_v9.h
AppData\Local\Temp\comfyui-diff.md:1189:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_ops.h
AppData\Local\Temp\comfyui-diff.md:1190:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_subquadratic_ops.h
AppData\Local\Temp\comfyui-diff.md:1191:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/include/cudnn_subquadratic_ops_v9.h
AppData\Local\Temp\comfyui-diff.md:1192:A /root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib
AppData\Local\Temp\comfyui-diff.md:1193:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_cnn.so.9
AppData\Local\Temp\comfyui-diff.md:1194:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_engines_precompiled.so.9
AppData\Local\Temp\comfyui-diff.md:1195:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_engines_runtime_compiled.so.9
AppData\Local\Temp\comfyui-diff.md:1196:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_engines_tensor_ir.so.9
AppData\Local\Temp\comfyui-diff.md:1197:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_graph.so.9
AppData\Local\Temp\comfyui-diff.md:1198:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_heuristic.so.9
AppData\Local\Temp\comfyui-diff.md:1199:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_ops.so.9
AppData\Local\Temp\comfyui-diff.md:1200:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn.so.9
AppData\Local\Temp\comfyui-diff.md:1201:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_adv.so.9
AppData\Local\Temp\comfyui-diff.md:1202:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia/cudnn/lib/libcudnn_ext.so.9
AppData\Local\Temp\comfyui-diff.md:1203:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info
AppData\Local\Temp\comfyui-diff.md:1204:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/top_level.txt
AppData\Local\Temp\comfyui-diff.md:1205:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/METADATA
AppData\Local\Temp\comfyui-diff.md:1206:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/RECORD
AppData\Local\Temp\comfyui-diff.md:1207:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/WHEEL
AppData\Local\Temp\comfyui-diff.md:1208:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/licenses
AppData\Local\Temp\comfyui-diff.md:1209:A
/root/.cache/uv/archive-v0/Y0IFmCtekTVJiwSFEcFyP/nvidia_cudnn_cu13-9.24.0.43.dist-info/licenses/License.txt
AppData\Local\Temp\comfyui-diff.md:1280:A /root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia
AppData\Local\Temp\comfyui-diff.md:1281:A /root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13
AppData\Local\Temp\comfyui-diff.md:1282:A /root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/include
AppData\Local\Temp\comfyui-diff.md:1283:A /root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/include/nvrtc.h
AppData\Local\Temp\comfyui-diff.md:1284:A /root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/lib
AppData\Local\Temp\comfyui-diff.md:1285:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/lib/libnvrtc.alt.so.13
AppData\Local\Temp\comfyui-diff.md:1286:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/lib/libnvrtc.so.13
AppData\Local\Temp\comfyui-diff.md:1287:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/lib/libnvrtc-builtins.alt.so.13.0
AppData\Local\Temp\comfyui-diff.md:1288:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia/cu13/lib/libnvrtc-builtins.so.13.0
AppData\Local\Temp\comfyui-diff.md:1289:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info
AppData\Local\Temp\comfyui-diff.md:1290:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/licenses
AppData\Local\Temp\comfyui-diff.md:1291:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/licenses/License.txt
AppData\Local\Temp\comfyui-diff.md:1292:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/top_level.txt
AppData\Local\Temp\comfyui-diff.md:1293:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/METADATA
AppData\Local\Temp\comfyui-diff.md:1294:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/RECORD
AppData\Local\Temp\comfyui-diff.md:1295:A
/root/.cache/uv/archive-v0/n5IFgk9ybF_nagDr-Ku4P/nvidia_cuda_nvrtc-13.0.88.dist-info/WHEEL
AppData\Local\Temp\comfyui-diff.md:1405:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia
AppData\Local\Temp\comfyui-diff.md:1406:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13
AppData\Local\Temp\comfyui-diff.md:1407:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include
AppData\Local\Temp\comfyui-diff.md:1408:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/cublasXt.h
AppData\Local\Temp\comfyui-diff.md:1409:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/cublas_api.h
AppData\Local\Temp\comfyui-diff.md:1410:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/cublas_v2.h
AppData\Local\Temp\comfyui-diff.md:1411:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/nvblas.h
AppData\Local\Temp\comfyui-diff.md:1412:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/cublas.h
AppData\Local\Temp\comfyui-diff.md:1413:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/include/cublasLt.h
AppData\Local\Temp\comfyui-diff.md:1414:A /root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/lib
AppData\Local\Temp\comfyui-diff.md:1415:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/lib/libcublasLt.so.13
AppData\Local\Temp\comfyui-diff.md:1416:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/lib/libnvblas.so.13
AppData\Local\Temp\comfyui-diff.md:1417:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia/cu13/lib/libcublas.so.13
AppData\Local\Temp\comfyui-diff.md:1418:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info
AppData\Local\Temp\comfyui-diff.md:1419:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/licenses
AppData\Local\Temp\comfyui-diff.md:1420:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/licenses/License.txt
AppData\Local\Temp\comfyui-diff.md:1421:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/top_level.txt
AppData\Local\Temp\comfyui-diff.md:1422:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/METADATA
AppData\Local\Temp\comfyui-diff.md:1423:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/RECORD
AppData\Local\Temp\comfyui-diff.md:1424:A
/root/.cache/uv/archive-v0/2jWnDwaUtjJvqaiFYoylW/nvidia_cublas-13.1.1.3.dist-info/WHEEL
AppData\Local\Temp\comfyui-diff.md:1867:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda
AppData\Local\Temp\comfyui-diff.md:1868:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/cpp_function_wrappers.cu
AppData\Local\Temp\comfyui-diff.md:1869:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator
AppData\Local\Temp\comfyui-diff.md:1870:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/kernelapi.py
AppData\Local\Temp\comfyui-diff.md:1871:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/reduction.py
AppData\Local\Temp\comfyui-diff.md:1872:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/vector_types.py
AppData\Local\Temp\comfyui-diff.md:1873:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/__init__.py
AppData\Local\Temp\comfyui-diff.md:1874:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/api.py
AppData\Local\Temp\comfyui-diff.md:1875:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/compiler.py
AppData\Local\Temp\comfyui-diff.md:1876:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv
AppData\Local\Temp\comfyui-diff.md:1877:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/devices.py
AppData\Local\Temp\comfyui-diff.md:1878:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/driver.py
AppData\Local\Temp\comfyui-diff.md:1879:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/drvapi.py
AppData\Local\Temp\comfyui-diff.md:1880:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/dummyarray.py
AppData\Local\Temp\comfyui-diff.md:1881:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/error.py
AppData\Local\Temp\comfyui-diff.md:1882:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/libs.py
AppData\Local\Temp\comfyui-diff.md:1883:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/nvvm.py
AppData\Local\Temp\comfyui-diff.md:1884:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/runtime.py
AppData\Local\Temp\comfyui-diff.md:1885:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/__init__.py
AppData\Local\Temp\comfyui-diff.md:1886:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/cudadrv/devicearray.py
AppData\Local\Temp\comfyui-diff.md:1887:A
/root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/simulator/kernel.py
AppData\Local\Temp\comfyui-diff.md:1888:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/target.py
AppData\Local\Temp\comfyui-diff.md:1889:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/api.py
AppData\Local\Temp\comfyui-diff.md:1890:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/api_util.py
AppData\Local\Temp\comfyui-diff.md:1891:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/nvvmutils.py
AppData\Local\Temp\comfyui-diff.md:1892:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests
AppData\Local\Temp\comfyui-diff.md:1893:A /root/.cache/uv/archive-v0/6-p6HgIiGG_spPaoVHiKe/numba/cuda/tests/nocuda
