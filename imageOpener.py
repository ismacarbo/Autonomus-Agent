import matplotlib.pyplot as plt
import imageio.v2 as imageio

img = imageio.imread('/tmp/robot_smoke_slam_map.pgm')

plt.imshow(img, cmap='gray')
plt.colorbar()
plt.savefig('robot_map.png', dpi=200, bbox_inches='tight')
print("Saved as robot_map.png")