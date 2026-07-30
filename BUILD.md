# LaserCNC 构建约定

本仓库只允许两棵生成树，两者互不混用：

| 路线 | 生成器 | 生成树 | 用途 |
| --- | --- | --- | --- |
| CMake | Ninja Multi-Config | `build-cmake/` | 命令行、CTest、日常开发和各硬件开关验证 |
| MSBuild | Visual Studio 17 2022 | `build-vs/` | `.sln`、Visual Studio IDE 和直接 MSBuild 构建 |

旧 `build/` 是曾被两种生成器共用的失效目录，禁止继续配置或打开其中的
解决方案。`CMakeLists.txt` 会拒绝在该目录生成项目。

两条路线是替代关系，不应并发构建。无论使用哪条路线，最终应用及其运行
依赖都只部署到根目录 `x64/Debug/` 或 `x64/Release/`。对象文件、静态库、
测试程序、PDB 和生成器元数据分别留在对应生成树中。

## CMake / Ninja 路线

```powershell
cmd /c "call \"C:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\Tools\VsDevCmd.bat\" -arch=x64 -host_arch=x64 && cmake --preset acs-gtn && cmake --build --preset acs-gtn-debug --parallel 16"
ctest --test-dir build-cmake --build-config Debug --output-on-failure
```

Release 构建：

```powershell
cmake --preset acs-gtn
cmake --build --preset acs-gtn-release --parallel 16
```

`all-off`、`acs`、`gtn` 和 `asan` 也使用 `build-cmake/`。切换配置 preset
时必须先重新运行相应 `cmake --preset <name>`，然后再运行对应 build preset。
Ninja 路线不得追加 MSBuild 的 `/m`、`/nologo` 等参数。

## Visual Studio / MSBuild 路线

生成或刷新解决方案：

```powershell
cmake --preset vs-acs-gtn
```

可直接打开 `build-vs/LaserCNC.sln`，也可以通过 CMake 驱动 MSBuild：

```powershell
cmake --build --preset vs-acs-gtn-debug --parallel 16
```

直接调用 MSBuild：

```powershell
& "E:\vs2022IDE\MSBuild\Current\Bin\MSBuild.exe" build-vs\LaserCNC.sln /m:16 /p:Configuration=Debug /p:Platform=x64
```

Release 将 `Debug` 改为 `Release`，或使用
`cmake --build --preset vs-acs-gtn-release --parallel 16`。
`/m`、`/nologo` 等参数只适用于这条路线。

## 清理旧生成树

先预览，再确认删除旧 `build/` 和历史生成目录：

```powershell
.\scripts\clean_legacy_build_artifacts.ps1
.\scripts\clean_legacy_build_artifacts.ps1 -Execute
```

该脚本不会删除 `build-cmake/`、`build-vs/` 或 `x64/`。

## 常见错误

- C1083 指向已删除源文件，同时出现 `generate.stamp`/MSB8065：正在使用旧
  `build/` 解决方案。关闭它并重新生成 `build-vs/LaserCNC.sln`。
- `__std_*` 未解析或 LNK4098：生成树中混入了不同 MSVC 工具集的旧对象；
  不要跨生成器复用目录，重新配置对应的独立生成树。
- Ninja 报 `unknown target '/m'`：把 MSBuild 参数传给了 Ninja。
- LNK1168/LNK1104 指向 `x64/<Config>/LaserCNC.exe`：应用仍在运行。不要在
  未经用户许可时强制终止，关闭应用后再链接。
