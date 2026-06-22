<div align="center">
    
# ![Robots](../docs/Images/robots-icon.svg)<br/>robots (Yaskawa Fork)
**Create and simulate ABB, KUKA, UR, Staubli, Franka Emika, Doosan, Fanuc, Igus, Jaka, and Yaskawa robot programs in Rhino 8, Grasshopper, and .NET**

[![License](https://img.shields.io/github/license/visose/Robots?style=flat-square)](../LICENSE)
[![NuGet package](https://img.shields.io/nuget/v/robots?style=flat-square)](https://www.nuget.org/packages?q=visose+robots)

**[About](#about) •
[Fork Features](#what-is-new-in-this-fork) •
[Install](#install) •
[Docs](../../../wiki) •
[Support](#support)**

</div>

---

## About

**Robots** is a plugin for **[Rhino 8](https://www.rhino3d.com/)** and the **Grasshopper** visual programming interface, with a .NET API for use in custom applications and automation. It lets users create, check, simulate, and save manufacturer-specific robot programs.

Supported manufacturers include **ABB**, **KUKA**, **UR**, **Staubli**, **Franka Emika**, **Doosan**, **Fanuc**, **Igus**, **Jaka**, and **Yaskawa**. Robots can load robot systems, tools, frames, and meshes from XML/3DM libraries, use custom post-processors, and generate manufacturer-specific robot code.

---

## What is new in this fork?

This fork introduces native compilation and post-processing support for **Yaskawa / Motoman** robotic arms. 

* **Yaskawa INFORM III Post-Processor:** Generates clean, structural `.JBI` files seamlessly mapped out of Grasshopper targets.
* **Kinematic Configuration Tracking:** Dynamically translates robot shoulder, elbow, and wrist flags into specific INFORM `SETE P[xxx] (7)` configuration bitmasks.
* **Inline Tool Selection:** Formats explicit tool calls directly within your motion targets (`T=1`, `T=2`, etc.).
* **Dynamic Joint Targets:** Supports dynamic, sequential array mapping to pulse registers directly through inline variables.

---

## Install

### Grasshopper plugin (Local Build)

Because this is a development fork featuring Yaskawa integration, it is best installed by cloning and compiling the source:

1. Clone this repository locally.
2. Build the solution using **.NET 8** to generate the compilation binaries.
3. Move the generated assemblies into your Grasshopper libraries folder.
4. Open **Rhino 8** and launch **Grasshopper**. There should be a new tab named `Robots`.
5. Install your custom Yaskawa/Motoman library by clicking on the `Libraries` button of a `Load robot system` component.

### .NET packages

- [`Robots`](https://www.nuget.org/packages/Robots) is the core .NET 8 package built on `Rhino3dm` for use outside Rhino.
- [`Robots.Rhino`](https://www.nuget.org/packages/Robots.Rhino) provides compile-time references for developing Rhino and Grasshopper plug-ins.

---

## Support

### 🆓 Community

For issues, troubleshooting, or discussions surrounding the Yaskawa post-processor implementation, please open an issue or start a thread directly inside this fork's repository tab!
