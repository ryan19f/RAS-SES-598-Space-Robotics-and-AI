# Photogrammetry and Gaussian Splatting on Lunar Apollo 17 imagery
## Part A
### Generate textured mesh model using Agisoft

1. Add Images
  ![add file](https://github.com/user-attachments/assets/40e41cd9-7a4e-45ca-865a-bfaf735ec674)
 
3. Align Images 
![align](https://github.com/user-attachments/assets/c4253e79-89eb-4480-85b8-4431874965c4)

4. Generate Point Cloud Model
   
![point clud](https://github.com/user-attachments/assets/f9f2369c-743f-4826-bf36-155562f5f0f5)

6. Generate  Model
![model](https://github.com/user-attachments/assets/115c7dc8-a1bd-4968-8017-1aa91b76b6a6)


7. Generate Texture Mesh Model
![texture](https://github.com/user-attachments/assets/2bda2a3a-5342-4f54-9a21-6c9d3ff6a4a8)

8. Export the Created Mesh
   
![export](https://github.com/user-attachments/assets/bf40b848-0f40-418f-aac7-f350ed33450a)

![modelk](https://github.com/user-attachments/assets/41af7429-f9e7-462b-8ea6-ac0f66781767)

## Gaussian Splatting using NeRFStudio

Use NeRFStudio software for Gaussian splatting

https://github.com/user-attachments/assets/edc8d8ab-bb56-45ub-95c2-a7cc4c1bac6d

#### Metrics  

The Gaussian Splatting model showed poor results. To improve performance, increasing image quality or quantity, and tuning training parameters may be needed.

PSNR : 14.13  
SSIM : 0.176


```
  "results": {
    "psnr": 14.136856575927734,
    "psnr_std": NaN,
    "ssim": 0.17671707847118378,
    "ssim_std": NaN,
    "lpips": 0.7272768020629883,
    "lpips_std": NaN,
    "num_rays_per_sec": 434151.90625,
    "num_rays_per_sec_std": NaN,
    "fps": 0.3144685924053192,
    "fps_std": NaN
  }
```





