import { Vector3, Color3, Quaternion } from "Maths/math";
import { Nullable } from "types";
import { Scene } from "scene";
import { TransformNode } from "Meshes/transformNode";
import { StandardMaterial } from "Materials/standardMaterial";
import { AxisDragGizmo } from "Gizmos/axisDragGizmo";

/**
     * The Axes viewer will show 3 axes in a specific point in space
     */
    export class AxesViewer {
        private _xAxis: TransformNode;
        private _yAxis: TransformNode;
        private _zAxis: TransformNode;
        private _tmpVector = new Vector3();
        private _scaleLinesFactor = 4;
        private _instanced = false;

        /**
         * Gets the hosting scene
         */
        public scene: Scene;

        /**
         * Gets or sets a number used to scale line length
         */
        public scaleLines = 1;

        /** Gets the node hierarchy used to render x-axis */
        public get xAxis(): TransformNode {
            return this._xAxis;
        }

        /** Gets the node hierarchy used to render y-axis */
        public get yAxis(): TransformNode {
            return this._yAxis;
        }

        /** Gets the node hierarchy used to render z-axis */
        public get zAxis(): TransformNode {
            return this._zAxis;
        }

        /**
         * Creates a new AxesViewer
         * @param scene defines the hosting scene
         * @param scaleLines defines a number used to scale line length (1 by default)
         * @param renderingGroupId defines a number used to set the renderingGroupId of the meshes (2 by default)
         * @param xAxis defines the node hierarchy used to render the x-axis
         * @param yAxis defines the node hierarchy used to render the y-axis
         * @param zAxis defines the node hierarchy used to render the z-axis
         */
        constructor(scene: Scene, scaleLines = 1, renderingGroupId: Nullable<number> = 2, xAxis?: TransformNode, yAxis?: TransformNode, zAxis?: TransformNode) {
            this.scaleLines = scaleLines;

            if (!xAxis) {
                var redColoredMaterial = new StandardMaterial("", scene);
                redColoredMaterial.disableLighting = true;
                redColoredMaterial.emissiveColor = Color3.Red().scale(0.5);
                xAxis = AxisDragGizmo._CreateArrow(scene, redColoredMaterial);
            }

            if (!yAxis) {
                var greenColoredMaterial = new StandardMaterial("", scene);
                greenColoredMaterial.disableLighting = true;
                greenColoredMaterial.emissiveColor = Color3.Green().scale(0.5);
                yAxis = AxisDragGizmo._CreateArrow(scene, greenColoredMaterial);
            }

            if (!zAxis) {
                var blueColoredMaterial = new StandardMaterial("", scene);
                blueColoredMaterial.disableLighting = true;
                blueColoredMaterial.emissiveColor = Color3.Blue().scale(0.5);
                zAxis = AxisDragGizmo._CreateArrow(scene, blueColoredMaterial);
            }

            this._xAxis = xAxis;
            this._xAxis.rotationQuaternion = new Quaternion();
            this._xAxis.scaling.setAll(this.scaleLines * this._scaleLinesFactor);
            this._yAxis = yAxis;
            this._yAxis.rotationQuaternion = new Quaternion();
            this._yAxis.scaling.setAll(this.scaleLines * this._scaleLinesFactor);
            this._zAxis = zAxis;
            this._zAxis.rotationQuaternion = new Quaternion();
            this._zAxis.scaling.setAll(this.scaleLines * this._scaleLinesFactor);

            if (renderingGroupId != null) {
                AxesViewer._SetRenderingGroupId(this._xAxis, renderingGroupId);
                AxesViewer._SetRenderingGroupId(this._yAxis, renderingGroupId);
                AxesViewer._SetRenderingGroupId(this._zAxis, renderingGroupId);
            }

            this.scene = scene;
            this.update(new Vector3(), Vector3.Right(), Vector3.Up(), Vector3.Forward());
        }

        /**
         * Force the viewer to update
         * @param position defines the position of the viewer
         * @param xaxis defines the x axis of the viewer
         * @param yaxis defines the y axis of the viewer
         * @param zaxis defines the z axis of the viewer
         */
        public update(position: Vector3, xaxis: Vector3, yaxis: Vector3, zaxis: Vector3): void {
            this._xAxis.position.copyFrom(position);
            xaxis.scaleToRef(-1, this._tmpVector);
            this._xAxis.setDirection(this._tmpVector);
            this._xAxis.scaling.setAll(this.scaleLines * this._scaleLinesFactor);

            this._yAxis.position.copyFrom(position);
            yaxis.scaleToRef(-1, this._tmpVector);
            this._yAxis.setDirection(this._tmpVector);
            this._yAxis.scaling.setAll(this.scaleLines * this._scaleLinesFactor);

            this._zAxis.position.copyFrom(position);
            zaxis.scaleToRef(-1, this._tmpVector);
            this._zAxis.setDirection(this._tmpVector);
            this._zAxis.scaling.setAll(this.scaleLines * this._scaleLinesFactor);
        }

        /**
         * Creates an instance of this axes viewer.
         * @returns a new axes viewer with instanced meshes
         */
        public createInstance(): AxesViewer {
            const xAxis = AxisDragGizmo._CreateArrowInstance(this.scene, this._xAxis);
            const yAxis = AxisDragGizmo._CreateArrowInstance(this.scene, this._yAxis);
            const zAxis = AxisDragGizmo._CreateArrowInstance(this.scene, this._zAxis);
            const axesViewer = new AxesViewer(this.scene, this.scaleLines, null, xAxis, yAxis, zAxis);
            axesViewer._instanced = true;
            return axesViewer;
        }

        /** Releases resources */
        public dispose() {
            if (this._xAxis) {
                this._xAxis.dispose(false, !this._instanced);
                delete this._xAxis;
            }

            if (this._yAxis) {
                this._yAxis.dispose(false, !this._instanced);
                delete this._yAxis;
            }

            if (this._zAxis) {
                this._zAxis.dispose(false, !this._instanced);
                delete this._zAxis;
            }

            delete this.scene;
        }

        private static _SetRenderingGroupId(node: TransformNode, id: number) {
            node.getChildMeshes().forEach((mesh) => {
                mesh.renderingGroupId = id;
            });
        }
    }
