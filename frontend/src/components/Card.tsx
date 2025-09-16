import React from "react";
import { Card as MuiCard, CardHeader, CardContent, Chip, Box } from "@mui/material";

type Props = {
  title: string;
  children: React.ReactNode;
  sourceLabel?: string; // "Carro" | "Central" | etc
  action?: React.ReactNode;
  height?: number;      // SE informado, fixa a altura do conteúdo
};

export default function Card({ title, children, sourceLabel, action, height }: Props) {
  return (
    <MuiCard>
      <CardHeader
        title={title}
        action={action}
        subheader={sourceLabel ? <Chip label={`Fonte: ${sourceLabel}`} size="small" /> : undefined}
        sx={{ pb: 0 }}
      />
      <CardContent sx={{ pt: 1 }}>
        {height ? <Box sx={{ height }}>{children}</Box> : children}
      </CardContent>
    </MuiCard>
  );
}
