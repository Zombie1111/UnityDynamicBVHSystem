# Simple Unity BVH Builder
Currently supports generating blas from array of triangels and tlas from array of blas instances.
Burst and job compatible.

Demo contains example generating BVH from a list of mesh colliders and ray intersection against BVH.

<img src="https://i.postimg.cc/8PqHpMTh/bvh-Top-View.png" width="100%" height="100%"/>
Performance with Burst one CPU, BVH construction: ~10.6ms | 10k rays: ~5ms

## Usage
**Dependencies (Likely workS in other versions)**
- Burst 1.8.21
- Collections 1.2.4
- Mathematics 1.2.6

**Installation**
Download .zip and copy API, Core, GameObjects and _Demo folders anywhere inside your Unity project assets folder

**Creating BVH**
- Pass array of triangels to BLASObject constructor
- Pass BLASObject to BLASInstance constructor along with its transform matrix
- Pass array of BLASInstance to TLASScene constructor
- Dont forget to dispose properly, see "GameObjects/GameObjectManager.cs" for code example

## License
MIT - see license.md for more details

