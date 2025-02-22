import streamlit as st
import pandas as pd
import numpy as np
import os

import plotly.express as px
import plotly.graph_objects as go
import seaborn as sns
import matplotlib.pyplot as plt

# Set page layout wide for more space
st.set_page_config(layout="wide")

def load_data(file):
    """
    Attempt to load a CSV file into a Pandas DataFrame.
    Return the DataFrame or None if loading fails.
    """
    try:
        df = pd.read_csv(file)
        return df
    except Exception as e:
        st.error(f"Error loading {file.name if hasattr(file,'name') else file}: {e}")
        return None


def identify_file_type(df):
    """
    Identify if a DataFrame corresponds to a Run (Run1 or Run2) or a Summary based on column checks.
    Return either 'run' or 'summary' or 'unknown'.
    """
    run_columns = {
        "TimeStep", "QueueN", "ArrivalsN", "AvgWaitN",
        "QueueE", "ArrivalsE", "AvgWaitE",
        "QueueS", "ArrivalsS", "AvgWaitS",
        "QueueW", "ArrivalsW", "AvgWaitW",
        "CrossingCount", "ProcessedCount", "OverallAvgWait"
    }
    summary_columns = {
        "SimulationRun", "JunctionType", "MultipleLights",
        "MultipleLanes", "LaneCount", "TotalVehiclesProcessed",
        "OverallAvgWait", "SimulateFullRoute", "RedEmptyN",
        "RedEmptyE", "RedEmptyS", "RedEmptyW",
        "CountCar", "CountTruck", "CountBus", "CountScooter", "CountMotorcycle"
    }

    df_cols = set(df.columns)

    if run_columns.issubset(df_cols):
        return "run"
    elif summary_columns.issubset(df_cols):
        return "summary"
    else:
        return "unknown"


def clean_run_data(df):
    """
    Clean and preprocess Run data:
    - Convert relevant columns to numeric.
    - Handle missing or inconsistent data.
    - Return cleaned DataFrame.
    """
    # Convert columns to numeric (if they're not already).
    numeric_cols = [
        "TimeStep", "QueueN", "ArrivalsN", "AvgWaitN",
        "QueueE", "ArrivalsE", "AvgWaitE",
        "QueueS", "ArrivalsS", "AvgWaitS",
        "QueueW", "ArrivalsW", "AvgWaitW",
        "CrossingCount", "ProcessedCount", "OverallAvgWait"
    ]
    for col in numeric_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # Drop rows where TimeStep is missing (or any major inconsistency).
    df.dropna(subset=["TimeStep"], inplace=True)

    # Sort by TimeStep for better time-series analysis
    df.sort_values(by="TimeStep", inplace=True)

    # Reset index
    df.reset_index(drop=True, inplace=True)
    return df


def clean_summary_data(df):
    """
    Clean and preprocess Summary data.
    """
    # Convert columns to numeric where possible
    numeric_cols = [
        "SimulationRun", "LaneCount", "TotalVehiclesProcessed",
        "OverallAvgWait", "RedEmptyN", "RedEmptyE", "RedEmptyS",
        "RedEmptyW", "CountCar", "CountTruck", "CountBus",
        "CountScooter", "CountMotorcycle"
    ]
    for col in numeric_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    # Sort if there's a 'SimulationRun' column
    if "SimulationRun" in df.columns:
        df.sort_values(by="SimulationRun", inplace=True)

    df.reset_index(drop=True, inplace=True)
    return df


def display_overview(df, file_label):
    """
    Display an overview of the data:
    - Columns
    - Number of rows
    - Number of missing values
    """
    st.subheader(f"Data Overview: {file_label}")
    st.write(f"**Columns**: {list(df.columns)}")
    st.write(f"**Number of rows**: {len(df)}")

    # Missing values
    missing_info = df.isnull().sum()
    st.write("**Missing values**:")
    st.write(missing_info[missing_info > 0])


def run_statistics(run_data, file_label):
    """
    Compute and display summary statistics for a run DataFrame:
    - Queue lengths
    - Arrivals
    - Average Waiting Time
    - Crossing Count & Processed Count
    - Overall Average Waiting Time
    """
    st.subheader(f"Statistical Analysis: {file_label}")

    queue_cols = ["QueueN","QueueE","QueueS","QueueW"]
    arrival_cols = ["ArrivalsN","ArrivalsE","ArrivalsS","ArrivalsW"]
    wait_cols = ["AvgWaitN","AvgWaitE","AvgWaitS","AvgWaitW"]
    crossing_cols = ["CrossingCount","ProcessedCount","OverallAvgWait"]

    # Utility function for summary
    def show_summary(col_list, title):
        st.markdown(f"**{title}**")
        st.dataframe(run_data[col_list].describe())

    show_summary(queue_cols, "Queue Lengths")
    show_summary(arrival_cols, "Arrivals")
    show_summary(wait_cols, "Average Waiting Times")
    show_summary(crossing_cols, "Crossing & Processing Stats")


def plot_time_series(run_data, file_label):
    """
    Time-Series Plots for queue lengths, arrivals, and waiting times over time.
    """
    st.subheader(f"Time-Series Plots: {file_label}")

    time_col = "TimeStep"

    # Queue Length Over Time
    fig_queue = go.Figure()
    for direction in ["N","E","S","W"]:
        col_name = f"Queue{direction}"
        if col_name in run_data.columns:
            fig_queue.add_trace(
                go.Scatter(x=run_data[time_col], y=run_data[col_name],
                           mode='lines', name=f"Queue{direction}")
            )
    fig_queue.update_layout(title="Queue Length Over Time", xaxis_title="TimeStep", yaxis_title="Queue Length")
    st.plotly_chart(fig_queue, use_container_width=True)

    # Arrivals Over Time
    fig_arrivals = go.Figure()
    for direction in ["N","E","S","W"]:
        col_name = f"Arrivals{direction}"
        if col_name in run_data.columns:
            fig_arrivals.add_trace(
                go.Scatter(x=run_data[time_col], y=run_data[col_name],
                           mode='lines', name=f"Arrivals{direction}")
            )
    fig_arrivals.update_layout(title="Arrivals Over Time", xaxis_title="TimeStep", yaxis_title="Arrivals")
    st.plotly_chart(fig_arrivals, use_container_width=True)

    # Average Waiting Time Over Time
    fig_wait = go.Figure()
    for direction in ["N","E","S","W"]:
        col_name = f"AvgWait{direction}"
        if col_name in run_data.columns:
            fig_wait.add_trace(
                go.Scatter(x=run_data[time_col], y=run_data[col_name],
                           mode='lines', name=f"AvgWait{direction}")
            )
    fig_wait.update_layout(title="Average Waiting Time Over Time", xaxis_title="TimeStep", yaxis_title="Waiting Time")
    st.plotly_chart(fig_wait, use_container_width=True)


def plot_histograms(run_data, file_label):
    """
    Histograms & Distributions: show frequency distributions of waiting times.
    """
    st.subheader(f"Histograms & Distributions: {file_label}")
    wait_cols = ["AvgWaitN","AvgWaitE","AvgWaitS","AvgWaitW"]
    # Create a combined waiting times column for easier distribution plot (optional)
    melted_wait = run_data[wait_cols].melt(value_name="WaitTime", var_name="Direction")

    fig_hist = px.histogram(melted_wait.dropna(), x="WaitTime", color="Direction", nbins=30,
                            title="Distribution of Waiting Times")
    st.plotly_chart(fig_hist, use_container_width=True)


def plot_heatmap(run_data, file_label):
    """
    Heatmap of correlation between variables.
    """
    st.subheader(f"Correlation Heatmap: {file_label}")
    # Select numeric columns only
    numeric_df = run_data.select_dtypes(include=[np.number])
    if len(numeric_df.columns) > 1:
        corr = numeric_df.corr()
        fig, ax = plt.subplots(figsize=(10,6))
        sns.heatmap(corr, annot=True, cmap="coolwarm", ax=ax)
        st.pyplot(fig)
    else:
        st.write("Not enough numeric data to plot a heatmap.")


def plot_boxplots(run_data, file_label):
    """
    Boxplots: Compare queue lengths and wait times across different directions.
    """
    st.subheader(f"Boxplots: {file_label}")
    directions = ["N","E","S","W"]
    queue_cols = [f"Queue{d}" for d in directions]
    wait_cols = [f"AvgWait{d}" for d in directions]

    # Melt for boxplot
    queue_melt = run_data[queue_cols].melt(value_name="QueueLength", var_name="Direction")
    wait_melt = run_data[wait_cols].melt(value_name="WaitTime", var_name="Direction")

    col1, col2 = st.columns(2)

    with col1:
        st.write("**Queue Lengths**")
        fig_q = px.box(queue_melt, x="Direction", y="QueueLength", title="Queue Length Comparison")
        st.plotly_chart(fig_q, use_container_width=True)

    with col2:
        st.write("**Waiting Times**")
        fig_w = px.box(wait_melt, x="Direction", y="WaitTime", title="Waiting Times Comparison")
        st.plotly_chart(fig_w, use_container_width=True)


def compute_aggregated_insights(run_data, file_label):
    """
    Aggregated Insights:
    - Peak traffic times based on queues & arrivals
    - Periods with longest waiting times
    - Traffic flow patterns
    - Moving averages for smoother trend analysis
    """
    st.subheader(f"Aggregated Insights: {file_label}")

    time_col = "TimeStep"

    # Peak traffic times (by sum of queues or sum of arrivals)
    run_data["TotalQueue"] = run_data["QueueN"] + run_data["QueueE"] + run_data["QueueS"] + run_data["QueueW"]
    run_data["TotalArrivals"] = run_data["ArrivalsN"] + run_data["ArrivalsE"] + run_data["ArrivalsS"] + run_data["ArrivalsW"]

    peak_queue_time = run_data.loc[run_data["TotalQueue"].idxmax(), time_col] if not run_data["TotalQueue"].empty else None
    peak_arrivals_time = run_data.loc[run_data["TotalArrivals"].idxmax(), time_col] if not run_data["TotalArrivals"].empty else None
    st.write(f"**Peak Queue Time**: {peak_queue_time}, Queue = {run_data['TotalQueue'].max()}")
    st.write(f"**Peak Arrivals Time**: {peak_arrivals_time}, Arrivals = {run_data['TotalArrivals'].max()}")

    # Longest waiting times (by sum of average waiting times, or by max among them)
    run_data["SumWait"] = run_data["AvgWaitN"] + run_data["AvgWaitE"] + run_data["AvgWaitS"] + run_data["AvgWaitW"]
    longest_wait_time = run_data.loc[run_data["SumWait"].idxmax(), time_col] if not run_data["SumWait"].empty else None
    st.write(f"**Longest Wait TimeStep**: {longest_wait_time}, Sum Wait = {run_data['SumWait'].max()}")

    # Moving average for smoother trend analysis (example using a window of 5)
    run_data["TotalQueue_MA5"] = run_data["TotalQueue"].rolling(window=5).mean()
    fig_ma = go.Figure()
    fig_ma.add_trace(
        go.Scatter(x=run_data[time_col], y=run_data["TotalQueue"], mode='lines', name="TotalQueue")
    )
    fig_ma.add_trace(
        go.Scatter(x=run_data[time_col], y=run_data["TotalQueue_MA5"], mode='lines', name="MA(5)")
    )
    fig_ma.update_layout(title="Moving Average (5) of Total Queue", xaxis_title="TimeStep", yaxis_title="Queue")
    st.plotly_chart(fig_ma, use_container_width=True)


def main():
    st.title("Traffic Simulation Data Analyzer (Run1, Run2, Summary)")
    st.write(
        """
        This application reads existing simulation CSV files (Run1, Run2, and/or Summary) and provides:
        - Data cleaning & preprocessing
        - Statistical analysis
        - Advanced visualizations
        - Aggregated insights (peak times, waiting patterns, etc.)
        
        **Note:** This script does *not* run new simulations; it only analyzes existing data.
        """
    )

    # Sidebar for file uploads
    st.sidebar.header("Upload CSV files")
    uploaded_files = st.sidebar.file_uploader(
        "Select one or more CSV files", 
        type=["csv"], 
        accept_multiple_files=True
    )

    if uploaded_files:
        # Dictionaries to store run data and summary data
        runs = []
        summaries = []

        for uploaded_file in uploaded_files:
            df = load_data(uploaded_file)
            if df is not None:
                ftype = identify_file_type(df)
                if ftype == "run":
                    df_cleaned = clean_run_data(df)
                    runs.append((uploaded_file.name, df_cleaned))
                elif ftype == "summary":
                    df_cleaned = clean_summary_data(df)
                    summaries.append((uploaded_file.name, df_cleaned))
                else:
                    st.warning(f"File '{uploaded_file.name}' not recognized as Run or Summary.")

        # Display data overviews, statistics, and visualizations
        for file_label, run_df in runs:
            st.markdown("---")
            display_overview(run_df, file_label)
            run_statistics(run_df, file_label)
            plot_time_series(run_df, file_label)
            plot_histograms(run_df, file_label)
            plot_heatmap(run_df, file_label)
            plot_boxplots(run_df, file_label)
            compute_aggregated_insights(run_df, file_label)

        # Summaries
        if summaries:
            st.markdown("---")
            st.header("Summary Data Analysis")
            for file_label, summary_df in summaries:
                st.subheader(f"Overview: {file_label}")
                display_overview(summary_df, file_label)
                st.dataframe(summary_df)


if __name__ == "__main__":
    main()
