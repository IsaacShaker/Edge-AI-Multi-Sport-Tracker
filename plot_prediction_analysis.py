#!/usr/bin/env python3
"""
Prediction Error Analysis Plot
Analyzes Kalman filter prediction accuracy from logged tracking data
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys
from pathlib import Path

def load_prediction_log(filepath='prediction_log.csv'):
    """Load prediction log CSV file"""
    try:
        df = pd.read_csv(filepath)
        print(f"Loaded {len(df)} prediction samples from {filepath}")
        return df
    except FileNotFoundError:
        print(f"Error: {filepath} not found!")
        print("Please run the tracker first to generate prediction data.")
        sys.exit(1)
    except Exception as e:
        print(f"Error loading data: {e}")
        sys.exit(1)

def calculate_statistics(df):
    """Calculate and print prediction error statistics"""
    print("\n=== Prediction Error Statistics ===")
    print(f"Total Frames:     {len(df)}")
    print(f"Mean Error:       {df['error'].mean():.2f} pixels")
    print(f"Median Error:     {df['error'].median():.2f} pixels")
    print(f"Std Dev:          {df['error'].std():.2f} pixels")
    print(f"Min Error:        {df['error'].min():.2f} pixels")
    print(f"Max Error:        {df['error'].max():.2f} pixels")
    print(f"95th Percentile:  {df['error'].quantile(0.95):.2f} pixels")
    print(f"Mean Velocity:    {df['velocity'].mean():.2f} px/s")
    print("===================================\n")

def plot_analysis(df, output_file='prediction_analysis.png'):
    """Create comprehensive prediction error analysis plots"""
    
    # Create figure with 6 subplots
    fig = plt.figure(figsize=(16, 10))
    
    # Plot 1: Prediction Error over Time
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(df['frame'], df['error'], linewidth=0.8, alpha=0.7)
    ax1.set_xlabel('Frame Number')
    ax1.set_ylabel('Prediction Error (pixels)')
    ax1.set_title('Kalman Filter Prediction Error Over Time')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=df['error'].mean(), color='r', linestyle='--', 
                label=f'Mean: {df["error"].mean():.2f}px', linewidth=2)
    ax1.legend()
    
    # Plot 2: Error Histogram
    ax2 = plt.subplot(2, 3, 2)
    ax2.hist(df['error'], bins=50, edgecolor='black', alpha=0.7)
    ax2.axvline(df['error'].mean(), color='r', linestyle='--', 
                label=f'Mean: {df["error"].mean():.2f}px', linewidth=2)
    ax2.axvline(df['error'].median(), color='g', linestyle='--', 
                label=f'Median: {df["error"].median():.2f}px', linewidth=2)
    ax2.set_xlabel('Prediction Error (pixels)')
    ax2.set_ylabel('Frequency')
    ax2.set_title(f'Error Distribution (σ={df["error"].std():.2f}px)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: X and Y Error Components
    ax3 = plt.subplot(2, 3, 3)
    error_x = np.abs(df['actual_x'] - df['predicted_x'])
    error_y = np.abs(df['actual_y'] - df['predicted_y'])
    ax3.plot(df['frame'], error_x, label='X Error', alpha=0.6, linewidth=0.8)
    ax3.plot(df['frame'], error_y, label='Y Error', alpha=0.6, linewidth=0.8)
    ax3.set_xlabel('Frame Number')
    ax3.set_ylabel('Error (pixels)')
    ax3.set_title('X and Y Error Components')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Velocity vs Error Scatter
    ax4 = plt.subplot(2, 3, 4)
    scatter = ax4.scatter(df['velocity'], df['error'], c=df['frame'], 
                         cmap='viridis', alpha=0.5, s=10)
    ax4.set_xlabel('Velocity Magnitude (px/s)')
    ax4.set_ylabel('Prediction Error (pixels)')
    ax4.set_title('Error vs Velocity')
    ax4.grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=ax4)
    cbar.set_label('Frame')
    
    # Add trend line
    z = np.polyfit(df['velocity'], df['error'], 1)
    p = np.poly1d(z)
    x_trend = np.linspace(df['velocity'].min(), df['velocity'].max(), 100)
    ax4.plot(x_trend, p(x_trend), "r--", alpha=0.8, linewidth=2, 
             label=f'Trend: y={z[0]:.3f}x+{z[1]:.2f}')
    ax4.legend()
    
    # Plot 5: Cumulative Error Distribution
    ax5 = plt.subplot(2, 3, 5)
    sorted_errors = np.sort(df['error'])
    cumulative = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors) * 100
    ax5.plot(sorted_errors, cumulative, linewidth=2)
    ax5.axvline(df['error'].quantile(0.95), color='r', linestyle='--', 
                label=f'95th percentile: {df["error"].quantile(0.95):.2f}px', linewidth=2)
    ax5.set_xlabel('Prediction Error (pixels)')
    ax5.set_ylabel('Cumulative Percentage (%)')
    ax5.set_title('Cumulative Error Distribution')
    ax5.grid(True, alpha=0.3)
    ax5.legend()
    
    # Plot 6: Trajectory Visualization (sample)
    ax6 = plt.subplot(2, 3, 6)
    # Plot every 5th point to avoid clutter
    sample_indices = range(0, len(df), 5)
    ax6.scatter(df['actual_x'].iloc[sample_indices], 
               df['actual_y'].iloc[sample_indices], 
               c='green', s=30, alpha=0.6, label='Actual Position')
    ax6.scatter(df['predicted_x'].iloc[sample_indices], 
               df['predicted_y'].iloc[sample_indices], 
               c='red', s=30, alpha=0.6, marker='x', label='Predicted Position')
    
    # Draw error vectors for a few samples
    sample_arrows = range(0, len(df), max(1, len(df) // 20))  # ~20 arrows
    for idx in sample_arrows:
        ax6.arrow(df['predicted_x'].iloc[idx], df['predicted_y'].iloc[idx],
                 df['actual_x'].iloc[idx] - df['predicted_x'].iloc[idx],
                 df['actual_y'].iloc[idx] - df['predicted_y'].iloc[idx],
                 head_width=5, head_length=7, fc='blue', ec='blue', alpha=0.3)
    
    ax6.set_xlabel('X Position (pixels)')
    ax6.set_ylabel('Y Position (pixels)')
    ax6.set_title('Predicted vs Actual Trajectory (sampled)')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    ax6.invert_yaxis()  # Invert Y axis to match image coordinates
    ax6.axis('equal')
    
    # Overall title
    fig.suptitle(f'Kalman Filter Prediction Analysis ({len(df)} frames)', 
                 fontsize=16, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved analysis plot to: {output_file}")
    plt.show()

def create_simple_plot(df, output_file='prediction_error_simple.png'):
    """Create a simple error-over-time plot for presentations"""
    
    fig, ax = plt.subplots(figsize=(12, 5))
    
    ax.plot(df['frame'], df['error'], linewidth=1.2, color='#2E86AB', alpha=0.8)
    ax.fill_between(df['frame'], 0, df['error'], alpha=0.2, color='#2E86AB')
    
    # Add mean line
    mean_error = df['error'].mean()
    ax.axhline(y=mean_error, color='#E63946', linestyle='--', 
               label=f'Mean Error: {mean_error:.2f} pixels', linewidth=2)
    
    ax.set_xlabel('Frame Number', fontsize=12)
    ax.set_ylabel('Prediction Error (pixels)', fontsize=12)
    ax.set_title('Kalman Filter Prediction Accuracy', fontsize=14, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Add stats text box
    stats_text = f'Mean: {df["error"].mean():.2f}px\n'
    stats_text += f'Median: {df["error"].median():.2f}px\n'
    stats_text += f'Std Dev: {df["error"].std():.2f}px\n'
    stats_text += f'95th %ile: {df["error"].quantile(0.95):.2f}px'
    
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved simple plot to: {output_file}")

def main():
    """Main function"""
    print("=== Kalman Filter Prediction Analysis ===")
    
    # Load data
    log_file = Path('prediction_log.csv')
    if not log_file.exists():
        # Try in server/build directory
        log_file = Path('server/build/prediction_log.csv')
    
    df = load_prediction_log(str(log_file))
    
    if len(df) == 0:
        print("No data to plot!")
        return
    
    # Calculate statistics
    calculate_statistics(df)
    
    # Create comprehensive analysis plot
    plot_analysis(df)
    
    # Create simple presentation plot
    create_simple_plot(df)
    
    print("\nAnalysis complete!")
    print("Generated plots:")
    print("  - prediction_analysis.png (comprehensive)")
    print("  - prediction_error_simple.png (presentation)")

if __name__ == '__main__':
    main()
