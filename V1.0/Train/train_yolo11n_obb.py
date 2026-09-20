from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import torch
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent
DEFAULT_DATA = ROOT / "data_ur5_obb.yaml"
DEFAULT_PROJECT = ROOT / "runs_ur5_obb"
DEFAULT_NAME = "ur5_yolo11n_obb_512_ultralight"
DEFAULT_MODEL = "yolo11n-obb.pt"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Train a lightweight YOLO11n OBB model for the UR5 scrub nurse dataset."
    )
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--patience", type=int, default=15)
    parser.add_argument("--name", default=DEFAULT_NAME)
    parser.add_argument("--project", type=Path, default=DEFAULT_PROJECT)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="Run a 1-epoch smoke test with lower batch/workers to validate training on GPU.",
    )
    parser.add_argument(
        "--skip-test",
        action="store_true",
        help="Skip final evaluation on the test split after training.",
    )
    return parser


def build_train_args(args: argparse.Namespace, batch: int, imgsz: int) -> dict[str, Any]:
    train_args: dict[str, Any] = {
        "data": str(args.data),
        "epochs": args.epochs,
        "imgsz": imgsz,
        "batch": batch,
        "device": args.device,
        "workers": args.workers,
        "patience": args.patience,
        "amp": True,
        "cache": False,
        "cos_lr": True,
        "close_mosaic": 10,
        "seed": args.seed,
        "project": str(args.project),
        "name": args.name,
        "degrees": 10.0,
        "translate": 0.05,
        "scale": 0.15,
        "fliplr": 0.5,
        "flipud": 0.0,
        "hsv_h": 0.0,
        "hsv_s": 0.0,
        "hsv_v": 0.0,
        "mixup": 0.0,
        "copy_paste": 0.0,
        "pretrained": True,
        "exist_ok": True,
        "verbose": True,
    }
    return train_args


def smoke_test_overrides(train_args: dict[str, Any]) -> dict[str, Any]:
    smoke_args = dict(train_args)
    smoke_args["epochs"] = 1
    smoke_args["batch"] = min(int(smoke_args["batch"]), 4)
    smoke_args["workers"] = min(int(smoke_args["workers"]), 2)
    smoke_args["name"] = f"{train_args['name']}_smoke"
    return smoke_args


def fallback_profiles(batch: int, imgsz: int) -> list[tuple[int, int]]:
    profiles = [(batch, imgsz)]
    if batch > 8:
        profiles.append((8, imgsz))
    if imgsz > 448:
        reduced_batch = min(batch, 8)
        profiles.append((reduced_batch, 448))

    unique_profiles: list[tuple[int, int]] = []
    for profile in profiles:
        if profile not in unique_profiles:
            unique_profiles.append(profile)
    return unique_profiles


def run_training(args: argparse.Namespace) -> Path:
    profiles = fallback_profiles(args.batch, args.imgsz)
    last_error: RuntimeError | None = None

    for attempt_idx, (batch, imgsz) in enumerate(profiles, start=1):
        print(f"[train] Attempt {attempt_idx}/{len(profiles)} with batch={batch}, imgsz={imgsz}")
        model = YOLO(args.model)
        try:
            model.train(**build_train_args(args, batch=batch, imgsz=imgsz))
            save_dir = Path(model.trainer.save_dir)
            print(f"[train] Training completed: {save_dir}")
            return save_dir
        except RuntimeError as exc:
            message = str(exc).lower()
            if "out of memory" not in message and "cuda" not in message:
                raise
            last_error = exc
            print(f"[train] CUDA issue detected, trying fallback profile. Error: {exc}")

    if last_error is not None:
        raise last_error
    raise RuntimeError("Training failed before any attempt could run.")


def run_smoke_test(args: argparse.Namespace) -> Path:
    print("[smoke] Starting 1-epoch GPU validation run.")
    model = YOLO(args.model)
    model.train(**smoke_test_overrides(build_train_args(args, batch=args.batch, imgsz=args.imgsz)))
    save_dir = Path(model.trainer.save_dir)
    print(f"[smoke] Smoke test completed: {save_dir}")
    return save_dir


def run_test_eval(model_path: Path, data_path: Path, device: str) -> None:
    print(f"[test] Evaluating best model on test split: {model_path}")
    model = YOLO(str(model_path))
    metrics = model.val(data=str(data_path), split="test", imgsz=512, batch=1, device=device, half=torch.cuda.is_available())
    print(f"[test] Test metrics: {metrics.results_dict}")


def main() -> None:
    args = build_parser().parse_args()

    if not args.data.exists():
        raise FileNotFoundError(f"Dataset YAML not found: {args.data}")

    print(f"[env] torch={torch.__version__}")
    print(f"[env] cuda_available={torch.cuda.is_available()}")
    print(f"[env] cuda_device_count={torch.cuda.device_count()}")
    if torch.cuda.is_available():
        print(f"[env] cuda_device_name={torch.cuda.get_device_name(0)}")
    else:
        raise RuntimeError("CUDA is not available in this environment. Install a CUDA-enabled PyTorch build first.")

    if args.smoke_test:
        run_smoke_test(args)
        return

    save_dir = run_training(args)
    best_model = save_dir / "weights" / "best.pt"
    if not best_model.exists():
        raise FileNotFoundError(f"Expected best.pt was not found: {best_model}")

    print(f"[done] Best model: {best_model}")
    print("[done] Recommended inference: imgsz=512, batch=1, conf=0.25, half=True on CUDA")

    if not args.skip_test:
        run_test_eval(best_model, args.data, args.device)


if __name__ == "__main__":
    main()
